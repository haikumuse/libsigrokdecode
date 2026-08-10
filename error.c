/*
 * This file is part of the libsigrokdecode project.
 *
 * Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>
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

#include "config.h"
#include "libsigrokdecode-internal.h"
#include "libsigrokdecode.h"
#include <glib.h>

/**
 * @file
 *
 * Error handling in libsigrokdecode.
 */

/*
 * Thread-local last-error message (restored from upstream).
 * This provides an alternative to the char **error out-parameter pattern.
 * Frontends can call srd_get_last_error() after any API function returns
 * a negative error code, instead of passing &error to every call.
 */
static GPrivate last_error_key = G_PRIVATE_INIT(g_free);

SRD_PRIV void srd_set_last_error(const char *msg)
{
	char *old = g_private_get(&last_error_key);
	g_free(old);
	g_private_replace(&last_error_key, msg ? g_strdup(msg) : NULL);
}

SRD_PRIV void srd_set_last_error_take(char *msg)
{
	char *old = g_private_get(&last_error_key);
	g_free(old);
	g_private_replace(&last_error_key, msg);
}

/**
 * Get the last error message (thread-local).
 *
 * The returned string is thread-local and remains valid until the next
 * libsigrokdecode call that sets an error in the current thread, or until
 * srd_clear_last_error() is called.
 *
 * @return The last error message, or NULL if no error occurred.
 *
 * @since 0.6.0
 */
SRD_API const char *srd_get_last_error(void)
{
	char *msg = g_private_get(&last_error_key);
	return msg;
}

/**
 * Clear the last error message (thread-local).
 *
 * @since 0.6.0
 */
SRD_API void srd_clear_last_error(void)
{
	char *old = g_private_get(&last_error_key);
	if (old) {
		g_free(old);
		g_private_replace(&last_error_key, NULL);
	}
}

/**
 * @defgroup grp_error Error handling
 *
 * Error handling in libsigrokdecode.
 *
 * libsigrokdecode functions usually return @ref SRD_OK upon success, or a
 * negative error code on failure.
 *
 * @{
 */

/**
 * Return a human-readable error string for the given libsigrokdecode error
 * code.
 *
 * @param error_code A libsigrokdecode error code number, such as
 *                   SRD_ERR_MALLOC.
 *
 * @return A const string containing a short, human-readable (English)
 *         description of the error, such as "memory allocation error".
 *         The string must NOT be free'd by the caller!
 *
 * @see srd_strerror_name
 *
 * @since 0.2.0
 */
SRD_API const char *srd_strerror(int error_code)
{
	const char *str;

	/*
	 * Note: All defined SRD_* error macros from libsigrokdecode.h must
	 * have an entry in this function, as well as in srd_strerror_name().
	 */

	switch (error_code) {
	case SRD_OK:
		str = "no error";
		break;
	case SRD_ERR:
		str = "generic/unspecified error";
		break;
	case SRD_ERR_MALLOC:
		str = "memory allocation error";
		break;
	case SRD_ERR_ARG:
		str = "invalid argument";
		break;
	case SRD_ERR_BUG:
		str = "internal error";
		break;
	case SRD_ERR_PYTHON:
		str = "Python API error";
		break;
	case SRD_ERR_DECODERS_DIR:
		str = "decoders directory access error";
		break;
	default:
		str = "unknown error";
		break;
	}

	return str;
}

/**
 * Return the "name" string of the given libsigrokdecode error code.
 *
 * For example, the "name" of the SRD_ERR_MALLOC error code is
 * "SRD_ERR_MALLOC", the name of the SRD_OK code is "SRD_OK", and so on.
 *
 * This function can be used for various purposes where the "name" string of
 * a libsigrokdecode error code is useful.
 *
 * @param error_code A libsigrokdecode error code number, such as
 *                   SRD_ERR_MALLOC.
 *
 * @return A const string containing the "name" of the error code as string.
 *         The string must NOT be free'd by the caller!
 *
 * @see srd_strerror
 *
 * @since 0.2.0
 */
SRD_API const char *srd_strerror_name(int error_code)
{
	const char *str;

	/*
	 * Note: All defined SRD_* error macros from libsigrokdecode.h must
	 * have an entry in this function, as well as in srd_strerror().
	 */

	switch (error_code) {
	case SRD_OK:
		str = "SRD_OK";
		break;
	case SRD_ERR:
		str = "SRD_ERR";
		break;
	case SRD_ERR_MALLOC:
		str = "SRD_ERR_MALLOC";
		break;
	case SRD_ERR_ARG:
		str = "SRD_ERR_ARG";
		break;
	case SRD_ERR_BUG:
		str = "SRD_ERR_BUG";
		break;
	case SRD_ERR_PYTHON:
		str = "SRD_ERR_PYTHON";
		break;
	case SRD_ERR_DECODERS_DIR:
		str = "SRD_ERR_DECODERS_DIR";
		break;
	default:
		str = "unknown error code";
		break;
	}

	return str;
}

/** @} */

/**
 * @defgroup grp_buildinfo Build info
 *
 * Build information querying functions.
 *
 * @{
 */

/**
 * Get a list of compile-time libraries and their versions.
 *
 * @return A GSList of "name version" strings. The caller must free the
 *         list and its contents with g_slist_free_full(list, g_free).
 *
 * @since 0.6.0
 */
SRD_API GSList *srd_buildinfo_libs_get(void)
{
	GSList *libs = NULL;

	libs = g_slist_append(libs,
		g_strdup_printf("glib %d.%d.%d",
			GLIB_MAJOR_VERSION, GLIB_MINOR_VERSION, GLIB_MICRO_VERSION));

	libs = g_slist_append(libs,
		g_strdup_printf("python %d.%d", PY_MAJOR_VERSION, PY_MINOR_VERSION));

#ifdef HAVE_LIBUSB_1_0
	libs = g_slist_append(libs, g_strdup("libusb-1.0"));
#endif

	return libs;
}

/**
 * Get the canonical host triplet for this build.
 *
 * @return A newly allocated string that the caller must free with g_free().
 *
 * @since 0.6.0
 */
SRD_API char *srd_buildinfo_host_get(void)
{
	return g_strdup(CONF_HOST);
}

/** @} */
