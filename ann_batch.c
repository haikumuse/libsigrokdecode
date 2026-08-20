/*
 * This file is part of the libsigrokdecode project.
 *
 * Batch annotation delivery (方案 E).
 *
 * Replaces the per-annotation callback + per-annotation g_malloc/g_free
 * hot path with a batch collector: annotations are appended into a
 * fixed-capacity item array (SRD_ANN_BATCH_MAX), payload strings are
 * copied into bump-allocated arena blocks, and the whole batch is
 * delivered to the host with a single callback call (array-at-once).
 *
 * Arena blocks are recycled through a per-session persistent pool so heap
 * calls are amortized instead of per-annotation. On flush the in-flight
 * block chain is moved into the pool for reuse; only blocks beyond
 * SRD_ANN_ARENA_POOL_CAP are g_free'd immediately. After warmup the only
 * remaining shared-heap calls on the decode hot path are: (a) g_malloc0
 * of a fresh block when the pool is empty (first burst / post-drain
 * spike), and (b) freeing a pooled block that is too small for an
 * oversized single allocation. Everything else is released once at
 * session destroy. This collapses millions of per-burst g_malloc/g_free
 * calls down to a handful, which removes the per-annotation heap-lock
 * (lock convoy) contention on the decode path.
 *
 * Copyright (C) 2026 PXView
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "config.h"
#include "libsigrokdecode-internal.h" /* First, so we avoid a _POSIX_C_SOURCE warning. */
#include "libsigrokdecode.h"
#include "log.h"
#include <glib.h>
#include <mimalloc.h>
#include <inttypes.h>
#include <string.h>

#define SRD_ANN_ARENA_BLOCK_CAP (64 * 1024)
/* Persistent pool high-water mark (blocks). Beyond this, excess blocks are
 * returned to the OS lazily on flush — bounds memory while keeping reuse. */
#define SRD_ANN_ARENA_POOL_CAP 256

SRD_PRIV void srd_ann_batch_init(struct srd_ann_batch_state *st)
{
	/* Per-session mimalloc heap: every arena block and the item array are
	 * allocated from here, so the decode thread's annotation memory never
	 * touches the shared process heap (no g_malloc/g_free on the hot path
	 * and no Win32 HeapCreate — cross-platform). */
	st->heap = mi_heap_new();
	st->items = mi_heap_zalloc(st->heap,
		SRD_ANN_BATCH_MAX * sizeof(struct srd_ann_item));
	st->n = 0;
	st->arena = NULL;
	st->pool = NULL;
	st->cb = NULL;
	st->cb_data = NULL;
	st->wrapper_installed = 0;
}

SRD_PRIV void srd_ann_batch_destroy(struct srd_ann_batch_state *st)
{
	if (!st)
		return;

	/* Deliver any remaining (typically none: threads are joined first). */
	if (st->n > 0)
		srd_ann_batch_flush_state(st);

	/* Bulk teardown: mi_heap_destroy frees the items array plus every arena
	 * block (in-flight chain AND persistent pool) in one pass — no per-block
	 * free, entirely off the shared process heap. */
	if (st->heap) {
		mi_heap_destroy(st->heap);
		st->heap = NULL;
	}
	st->items = NULL;
	st->arena = NULL;
	st->pool = NULL;
	st->n = 0;
}

/* 8-byte aligned bump allocation from the current arena block.
 * When the current block is exhausted, first reuse a block from the
 * persistent pool (no heap interaction); only grow the pool via the
 * session's mimalloc heap when the pool is empty. */
SRD_PRIV void *srd_ann_arena_alloc(struct srd_ann_batch_state *st, size_t n)
{
	struct srd_ann_arena_block *blk;
	size_t aligned = (n + 7) & ~(size_t)7;
	size_t cap;

	blk = st->arena;
	if (!blk || blk->used + aligned > blk->cap) {
		cap = aligned > SRD_ANN_ARENA_BLOCK_CAP ?
			aligned : SRD_ANN_ARENA_BLOCK_CAP;
		blk = st->pool;
		if (blk) {
			st->pool = blk->next;
			if (blk->cap < aligned) {
				/* Oversized single allocation: drop the pooled
				 * block and allocate a fresh, larger one. */
				mi_free(blk);
				blk = mi_heap_zalloc(st->heap,
					sizeof(struct srd_ann_arena_block) + cap);
				blk->cap = cap;
			}
			blk->used = 0;
			blk->next = NULL;
		} else {
			blk = mi_heap_zalloc(st->heap,
				sizeof(struct srd_ann_arena_block) + cap);
			blk->cap = cap;
			blk->used = 0;
			blk->next = NULL;
		}
		/* head-insert into the in-flight chain */
		blk->next = st->arena;
		st->arena = blk;
	}

	{
		void *p = blk->data + blk->used;
		blk->used += aligned;
		return p;
	}
}

SRD_PRIV char *srd_ann_arena_strdup(struct srd_ann_batch_state *st, const char *s)
{
	size_t len = strlen(s);
	char *dst = srd_ann_arena_alloc(st, len + 1);

	memcpy(dst, s, len + 1);
	return dst;
}

SRD_PRIV void srd_ann_batch_append_item(struct srd_ann_batch_state *st,
		const struct srd_ann_item *it)
{
	if (st->n >= SRD_ANN_BATCH_MAX)
		srd_ann_batch_flush_state(st);   /* defensive */

	st->items[st->n++] = *it;

	if (st->n >= SRD_ANN_BATCH_MAX)
		srd_ann_batch_flush_state(st);
}

SRD_PRIV void srd_ann_batch_append_fields(struct srd_ann_batch_state *st,
		uint64_t start, uint64_t end, int ann_class, int ann_type,
		const struct srd_decoder *decoder,
		const char *const *ann_text, const char *hex, long long numeric)
{
	struct srd_ann_item it;
	const char *const *p;
	const char **strv;
	size_t nstr = 0;
	size_t i;

	if (ann_text) {
		for (p = ann_text; *p; p++)
			nstr++;
	}

	/* NULL-terminated strv array allocated from the arena. */
	strv = srd_ann_arena_alloc(st, (nstr + 1) * sizeof(char *));
	for (i = 0; i < nstr; i++)
		strv[i] = srd_ann_arena_strdup(st, ann_text[i]);
	strv[nstr] = NULL;

	memset(&it, 0, sizeof(it));
	it.start_sample = start;
	it.end_sample = end;
	it.ann_class = ann_class;
	it.ann_type = ann_type;
	it.ann_text = (const char *const *)strv;
	it.numberic_value = numeric;
	it.decoder = decoder;
	if (hex)
		memcpy(it.str_number_hex, hex, DECODE_NUM_HEX_MAX_LEN);

	srd_ann_batch_append_item(st, &it);
}

SRD_PRIV void srd_ann_batch_flush_state(struct srd_ann_batch_state *st)
{
	struct srd_ann_arena_block *blk, *next;

	if (st->n == 0)
		return;

	{
		struct srd_ann_batch b = { st->items, st->n };
		if (st->cb)
			st->cb(&b, st->cb_data);
	}

	/* Persistent pool: move the in-flight chain into the pool for reuse
	 * (release is deferred — only excess blocks beyond the cap are returned
	 * to the OS here; the whole pool is freed at session destroy via
	 * mi_heap_destroy). This removes the per-flush free from the decode hot
	 * path entirely. */
	{
		size_t pool_cnt = 0;

		for (blk = st->pool; blk; blk = blk->next)
			pool_cnt++;

		blk = st->arena;
		st->arena = NULL;
		while (blk) {
			next = blk->next;
			if (pool_cnt < SRD_ANN_ARENA_POOL_CAP) {
				blk->next = st->pool;
				st->pool = blk;
				pool_cnt++;
			} else {
				mi_free(blk);
			}
			blk = next;
		}
	}
	st->n = 0;
}

/* Registered as the actual SRD_OUTPUT_ANN callback; cb_data is the session. */
SRD_PRIV void srd_ann_batch_callback_wrapper(struct srd_proto_data *pdata,
		void *cb_data)
{
	struct srd_session *sess;
	struct srd_proto_data_annotation *pda;

	if (!pdata)
		return;
	sess = cb_data;
	if (!sess)
		return;
	pda = pdata->data;
	if (!pda)
		return;

	/* 携带产生该注解的解码器身份，供前端按 it->decoder 精确路由到行。 */
	const struct srd_decoder *decoder = NULL;
	if (pdata->pdo && pdata->pdo->di)
		decoder = pdata->pdo->di->decoder;

	srd_ann_batch_append_fields(&sess->ann_batch,
			pdata->start_sample, pdata->end_sample,
			pda->ann_class, pda->ann_type, decoder,
			(const char *const *)pda->ann_text,
			pda->str_number_hex, pda->numberic_value);
}

SRD_API void srd_ann_batch_flush(struct srd_session *sess)
{
	if (sess)
		srd_ann_batch_flush_state(&sess->ann_batch);
}
