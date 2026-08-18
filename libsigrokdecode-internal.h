/*
 * This file is part of the libsigrokdecode project.
 *
 * Copyright (C) 2011 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2012 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2016 DreamSourceLab <support@dreamsourcelab.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROKDECODE_LIBSIGROKDECODE_INTERNAL_H
#define LIBSIGROKDECODE_LIBSIGROKDECODE_INTERNAL_H

/*
 * Use the stable ABI subset as per PEP 384.
 *
 * Free-threaded Python (PEP 703, Python 3.13t+) does NOT support the
 * limited API — Python.h emits a hard #error if Py_LIMITED_API is defined.
 * SRD_FREE_THREADED_PYTHON is set by CMake (deps.cmake) when it detects
 * Py_GIL_DISABLED in the Python headers at configure time. When set,
 * we use the full C API instead of the limited ABI.
 *
 * Note: We cannot check Py_GIL_DISABLED directly here because it is
 * defined by pyconfig.h which is included BY Python.h — i.e., not yet
 * defined at this point in the file.
 */
#ifndef SRD_FREE_THREADED_PYTHON
# define Py_LIMITED_API 0x03020000  /* Python 3.2 stable ABI */
#endif

#include <Python.h> /* First, so we avoid a _POSIX_C_SOURCE warning. */
#include "libsigrokdecode.h"
#include <structmember.h>

#define safe_free(p) if((p)){free((p)); (p) = NULL;}

/*
 * Static definition of tables ending with an all-zero sentinel entry
 * may raise warnings when compiling with -Wmissing-field-initializers.
 * GCC suppresses the warning only with { 0 }, clang wants { } instead.
 */
#ifdef __clang__
#  define ALL_ZERO { }
#else
#  define ALL_ZERO { 0 }
#endif

/* srd_term_type enum and struct srd_term are now in libsigrokdecode.h */

/*
 * struct srd_session is defined here (PRIVATE) so that the public header
 * only has a forward declaration. Frontends must use the API functions.
 */

/* ---- Batch annotation delivery (方案 E) ---- */
struct srd_ann_arena_block {
	struct srd_ann_arena_block *next;
	size_t used;
	size_t cap;
	unsigned char data[];   /* flexible array member */
};

struct srd_ann_batch_state {
	srd_pd_output_batch_callback cb;   /* 宿主批量回调 */
	void *cb_data;
	struct srd_ann_item *items;        /* SRD_ANN_BATCH_MAX 容量，init 时分配一次 */
	size_t n;
	struct srd_ann_arena_block *arena; /* 当前批次使用的块链（flush 后入池复用） */
	struct srd_ann_arena_block *pool;  /* 每线程/每会话持久池：跨批次复用，惰性归还 OS */
	int wrapper_installed;             /* 批处理包装器是否已装到 sess->callbacks */
};

SRD_PRIV void srd_ann_batch_init(struct srd_ann_batch_state *st);
SRD_PRIV void srd_ann_batch_destroy(struct srd_ann_batch_state *st);
SRD_PRIV void *srd_ann_arena_alloc(struct srd_ann_batch_state *st, size_t n);
SRD_PRIV char *srd_ann_arena_strdup(struct srd_ann_batch_state *st, const char *s);
SRD_PRIV void srd_ann_batch_append_fields(struct srd_ann_batch_state *st,
		uint64_t start, uint64_t end, int ann_class, int ann_type,
		const struct srd_decoder *decoder,
		const char *const *ann_text, const char *hex, long long numeric);
SRD_PRIV void srd_ann_batch_append_item(struct srd_ann_batch_state *st,
		const struct srd_ann_item *it);
SRD_PRIV void srd_ann_batch_flush_state(struct srd_ann_batch_state *st);
SRD_PRIV void srd_ann_batch_callback_wrapper(struct srd_proto_data *pdata,
		void *cb_data);

struct srd_session {
	int session_id;

	/* List of decoder instances. srd_decoder_inst* type */
	GSList *di_list;

	/* List of frontend callbacks to receive decoder output. */
	GSList *callbacks;

	struct srd_ann_batch_state ann_batch;
};

/* Custom Python types: */

typedef struct {
	PyObject_HEAD
	struct srd_decoder_inst *di;
	uint64_t abs_start_samplenum;
	unsigned int itercnt;
	uint8_t *inbuf;
	uint64_t inbuflen;
	PyObject *sample;
} srd_logic;

/* srd.c */
SRD_PRIV int srd_decoder_searchpath_add(const char *path);

/* session.c */
SRD_PRIV struct srd_pd_callback *srd_pd_output_callback_find(struct srd_session *sess,
		int output_type);

/* instance.c */
SRD_PRIV int srd_inst_start(struct srd_decoder_inst *di, char **error);
SRD_PRIV void condition_list_free(struct srd_decoder_inst *di);
SRD_PRIV int srd_inst_decode(struct srd_decoder_inst *di,
        uint64_t abs_start_samplenum, uint64_t abs_end_samplenum,
        const uint8_t **inbuf, const uint8_t *inbuf_const, uint64_t inbuflen, char **error);
SRD_PRIV int process_samples_until_condition_match(struct srd_decoder_inst *di, gboolean *found_match);
SRD_PRIV int srd_inst_terminate_reset(struct srd_decoder_inst *di);
SRD_PRIV void srd_inst_free(struct srd_decoder_inst *di);
SRD_PRIV void srd_inst_free_all(struct srd_session *sess);
SRD_PRIV struct srd_decoder_inst *create_c_decoder_inst(struct srd_session *sess,
		struct srd_decoder *dec, GHashTable *options);

/* log.c — restored callback mechanism (compatible with xlog) */
#if defined(G_OS_WIN32) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4))
SRD_PRIV int srd_log(int loglevel, const char *format, ...)
		__attribute__((__format__ (__gnu_printf__, 2, 3)));
#else
SRD_PRIV int srd_log(int loglevel, const char *format, ...) G_GNUC_PRINTF(2, 3);
#endif

/* error.c — thread-local last error (for srd_get_last_error()) */
SRD_PRIV void srd_set_last_error(const char *msg);
SRD_PRIV void srd_set_last_error_take(char *msg);

/*
 * Instance operations vtable (PRIVATE).
 *
 * This vtable abstracts the C/Python decoder dispatch. Each decoder
 * instance stores a pointer to one of these, and the framework calls
 * through the vtable instead of branching on di->is_c_inst.
 *
 * The C implementation calls c_dec_inst->start/reset/end/etc.
 * The Python implementation calls PyObject_CallMethod.
 */
struct srd_inst_ops {
	/* Call the decoder's start() method */
	int  (*call_start)(struct srd_decoder_inst *di, char **error);

	/* Call the decoder's metadata() method */
	void (*call_metadata)(struct srd_decoder_inst *di, int key, uint64_t value);

	/* Call the decoder's end() method */
	int  (*call_end)(struct srd_decoder_inst *di, char **error);

	/* Call the decoder's reset() method (for terminate_reset) */
	void (*call_reset)(struct srd_decoder_inst *di);

	/* Free decoder-specific resources (called by srd_inst_free) */
	void (*free_resources)(struct srd_decoder_inst *di);

	/* Set options on the decoder */
	int  (*option_set)(struct srd_decoder_inst *di, GHashTable *options);

	/* Worker thread function */
	gpointer (*decode_thread)(gpointer data);

	/* Join/cleanup the worker thread */
	void (*join_thread)(struct srd_decoder_inst *di);

	/* Extract error message (transfer ownership, may return NULL) */
	char *(*extract_error)(struct srd_decoder_inst *di);
};

extern const struct srd_inst_ops c_inst_ops;
extern const struct srd_inst_ops py_inst_ops;

/* Helper: retrieve the ops vtable from a decoder instance */
static inline const struct srd_inst_ops *srd_di_ops(const struct srd_decoder_inst *di)
{
	return (const struct srd_inst_ops *)di->ops;
}

/* decoder.c */
SRD_PRIV long srd_decoder_apiver(const struct srd_decoder *d);

/* Free-threaded Python (PEP 703) support: mutex protecting the global
 * pd_list in decoder.c. Previously the GIL serialised all accesses to
 * pd_list (srd_decoder_load, srd_decoder_unload, srd_decoder_list,
 * srd_decoder_get_by_id). Without a GIL, explicit locking is required.
 * The mutex is defined in decoder.c and initialised in srd_init(). */
SRD_PRIV extern GMutex pd_list_mutex;

/* type_decoder.c */
SRD_PRIV PyObject *srd_Decoder_type_new(void);
SRD_PRIV const char *output_type_name(unsigned int idx);

/* type_logic.c */
SRD_PRIV PyObject *srd_logic_type_new(void);

/* module_sigrokdecode.c */
PyMODINIT_FUNC PyInit_sigrokdecode(void);
extern SRD_PRIV PyObject *srd_ChunkDone_exc;

/* util.c */
SRD_PRIV PyObject *py_import_by_name(const char *name);
SRD_PRIV int py_attr_as_str(PyObject *py_obj, const char *attr, char **outstr);
SRD_PRIV int py_attr_as_strlist(PyObject *py_obj, const char *attr, GSList **outstrlist);
SRD_PRIV int py_dictitem_as_str(PyObject *py_obj, const char *key, char **outstr);
SRD_PRIV int py_dictitem_to_int(PyObject *py_obj, const char *key);
SRD_PRIV int py_listitem_as_str(PyObject *py_obj, int idx, char **outstr);
SRD_PRIV int py_dict_value_to_str(PyObject *py_obj, PyObject *py_key, char **outstr);
SRD_PRIV int py_pydictitem_as_long(PyObject *py_obj, PyObject *py_key, uint64_t *out);
SRD_PRIV int py_str_as_str(PyObject *py_str, char **outstr);
SRD_PRIV int py_strseq_to_char(PyObject *py_strseq, char ***out_strv);
SRD_PRIV GVariant *py_obj_to_variant(PyObject *py_obj);

/*
	python string object to c string, free by g_free()
	if success, return 0;
*/
#define py_object_to_str_alloc py_str_as_str

SRD_PRIV int py_object_to_int(PyObject *py_obj,int64_t *out);

SRD_PRIV int py_object_to_uint(PyObject *py_obj,uint64_t *out);

/* exception.c */
#if defined(G_OS_WIN32) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4))
/*
 * On MinGW, we need to specify the gnu_printf format flavor or GCC
 * will assume non-standard Microsoft printf syntax.
 */
SRD_PRIV void srd_exception_catch(char **error, const char *format, ...)
		__attribute__((__format__ (__gnu_printf__, 2, 3)));
#else
SRD_PRIV void srd_exception_catch(char **error, const char *format, ...) G_GNUC_PRINTF(2, 3);
#endif

#endif
