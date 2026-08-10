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

#ifndef _SRD_LOG_H_
#define _SRD_LOG_H_

#include "libsigrokdecode.h"
#include <log/xlog.h>
#include <glib.h>

extern xlog_writer *srd_xlog;

/**
 * Init a private log context.
 */
SRD_PRIV void srd_log_init(void);

/**
 * Destroy the private log context.
 */
SRD_PRIV void srd_log_uninit(void);

/**
 * Internal log dispatch function.
 *
 * Routes messages to either a custom callback (if set via
 * srd_log_callback_set()) or the xlog backend (default).
 */
#if defined(G_OS_WIN32) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4))
SRD_PRIV int srd_log(int loglevel, const char *format, ...)
		__attribute__((__format__ (__gnu_printf__, 2, 3)));
#else
SRD_PRIV int srd_log(int loglevel, const char *format, ...) G_GNUC_PRINTF(2, 3);
#endif

/*
 * Convenience macros — all library code uses these.
 * They route through srd_log(), which supports both the xlog backend
 * (default) and custom callbacks (set via srd_log_callback_set()).
 *
 * SRD_LOG_* levels are defined in libsigrokdecode.h (enum srd_loglevel).
 */
#define LOG_PREFIX ""

#define srd_err(...)  srd_log(SRD_LOG_ERR,  LOG_PREFIX __VA_ARGS__)
#define srd_warn(...) srd_log(SRD_LOG_WARN, LOG_PREFIX __VA_ARGS__)
#define srd_info(...) srd_log(SRD_LOG_INFO, LOG_PREFIX __VA_ARGS__)

#ifndef NDEBUG
#define srd_dbg(...)  srd_log(SRD_LOG_DBG,  LOG_PREFIX __VA_ARGS__)
#define srd_spew(...) srd_log(SRD_LOG_SPEW, LOG_PREFIX __VA_ARGS__)
#else
#define srd_dbg(...)  ((void)0)
#define srd_spew(...) ((void)0)
#endif

/*
 * Backward compatibility: srd_detail was the old name for the
 * highest verbosity level (now SRD_LOG_SPEW).
 */
#define srd_detail(...) srd_spew(__VA_ARGS__)

#endif
