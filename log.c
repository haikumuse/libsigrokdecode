/*
 * This file is part of the PXView project.
 * PXView is based on DSView.
 * PXView is based on PulseView.
 * 
 * Copyright (C) 2022 DreamSourceLab <support@dreamsourcelab.com>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */
  #include "log.h"
#include <stdarg.h>
 #define LOG_DOMAIN  "srd"
 xlog_writer *srd_log = NULL;
static xlog_context *log_ctx = NULL; /* private log context */
static int is_private_log = 0;
static int log_level_value = SRD_LOG_WARN;
 /* --- Callback mechanism (restored from upstream) --- */
 static srd_log_callback user_log_cb = NULL;
static void *user_log_cb_data = NULL;
 /*
 * Default callback: routes messages through the xlog backend.
 * This is used when no custom callback is set.
 */
static int default_log_callback(void *cb_data, int loglevel,
								const char *format, va_list args)
{
	xlog_writer *wr = (xlog_writer *)cb_data;
	char *msg;
 	if (!wr)
		return SRD_ERR_ARG;
 	msg = g_strdup_vprintf(format, args);
 	switch (loglevel) {
	case SRD_LOG_ERR:
		xlog_err(wr, "%s", msg);
		break;
	case SRD_LOG_WARN:
		xlog_warn(wr, "%s", msg);
		break;
	case SRD_LOG_INFO:
		xlog_info(wr, "%s", msg);
		break;
	case SRD_LOG_DBG:
		xlog_dbg(wr, "%s", msg);
		break;
	case SRD_LOG_SPEW:
		xlog_detail(wr, "%s", msg);
		break;
	default:
		break;
	}
 	g_free(msg);
	return SRD_OK;
}
 /**
 * Internal log dispatch function.
 *
 * Routes the message to the currently active log callback:
 * - If a custom callback was set via srd_log_callback_set(), it is used.
 * - Otherwise, the default callback routes through the xlog backend.
 *
 * Messages above the current log level are silently dropped.
 */
SRD_PRIV int srd_log(int loglevel, const char *format, ...)
{
	va_list args;
 	if (loglevel > log_level_value)
		return SRD_OK;
 	if (!srd_log && !user_log_cb)
		return SRD_OK; /* Not initialized yet, silently drop. */
 	va_start(args, format);
 	if (user_log_cb)
		user_log_cb(user_log_cb_data, loglevel, format, args);
	else
		default_log_callback(srd_log, loglevel, format, args);
 	va_end(args);
 	return SRD_OK;
}
 /**
 * Init a private log context.
 */
SRD_PRIV void srd_log_init(void)
{
	if (!log_ctx && !srd_log) {
		log_ctx = xlog_new();
		srd_log = xlog_create_writer(log_ctx, LOG_DOMAIN);
		is_private_log = 1;
 		if (log_level_value != SRD_LOG_WARN)
			xlog_set_level(log_ctx, log_level_value);
	}
}
 /**
 * Destroy the private log context.
 */
SRD_PRIV void srd_log_uninit(void)
{
	if (is_private_log && log_ctx) {
		xlog_free(log_ctx);
		log_ctx = NULL;
		xlog_free_writer(srd_log);
		srd_log = NULL;
		is_private_log = 0;
	}
}
 /**
 * Use a shared xlog context, and drop the private log context.
 */
SRD_API void srd_log_set_context(xlog_context *ctx)
{
	if (ctx) {
		srd_log_uninit();
		srd_log = xlog_create_writer(ctx, LOG_DOMAIN);
 		if (log_level_value != SRD_LOG_WARN)
			xlog_set_level(ctx, log_level_value);
	}
}
 /**
 * Set the log level.
 *
 * This maps to both the xlog backend level and the callback filter level.
 * When a custom callback is active, messages above this level are not
 * dispatched to the callback.
 *
 * @param loglevel The new log level (see enum srd_loglevel).
 *
 * @return SRD_OK upon success, SRD_ERR_ARG upon invalid level.
 *
 * @since 0.6.0
 */
SRD_API int srd_log_loglevel_set(int loglevel)
{
	if (loglevel < SRD_LOG_NONE || loglevel > SRD_LOG_SPEW)
		return SRD_ERR_ARG;
 	log_level_value = loglevel;
 	if (log_ctx)
		xlog_set_level(log_ctx, loglevel);
 	return SRD_OK;
}
 /**
 * Get the current log level.
 *
 * @return The current log level.
 *
 * @since 0.6.0
 */
SRD_API int srd_log_loglevel_get(void)
{
	return log_level_value;
}
 /**
 * Set a custom log callback.
 *
 * Once set, all log messages are routed through this callback instead of
 * the xlog backend. Pass NULL to restore the default (xlog) backend.
 *
 * @param cb The callback function, or NULL to restore the default.
 * @param cb_data User data pointer passed to the callback.
 *
 * @return SRD_OK upon success.
 *
 * @since 0.6.0
 */
SRD_API int srd_log_callback_set(srd_log_callback cb, void *cb_data)
{
	user_log_cb = cb;
	user_log_cb_data = cb_data;
	return SRD_OK;
}
 /**
 * Get the current log callback and its user data.
 *
 * @param cb Output pointer for the callback function (may be NULL).
 * @param cb_data Output pointer for the user data (may be NULL).
 *
 * @return SRD_OK upon success.
 *
 * @since 0.6.0
 */
SRD_API int srd_log_callback_get(srd_log_callback *cb, void **cb_data)
{
	if (cb)
		*cb = user_log_cb;
	if (cb_data)
		*cb_data = user_log_cb_data;
	return SRD_OK;
}
 /**
 * Restore the default log callback (xlog backend).
 *
 * @return SRD_OK upon success.
 *
 * @since 0.6.0
 */
SRD_API int srd_log_callback_set_default(void)
{
	user_log_cb = NULL;
	user_log_cb_data = NULL;
	return SRD_OK;
}
 /*
 * Backward compatibility wrapper for the old srd_log_level() API.
 * New code should use srd_log_loglevel_set() instead.
 */
SRD_API void srd_log_level(int level)
{
	srd_log_loglevel_set(level);
}
