/*
 *  Copyright (C)
 *    2011 - 2012  Arnaud Quette <arnaud.quette@free.fr>
 *    2016 - 2021  EATON - Various threads-related improvements
 *    2020 - 2024  Jim Klimov <jimklimov+nut@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*! \file scan_ipmi.c
    \brief detect NUT supported Power Supply Units
    \author Arnaud Quette <arnaud.quette@free.fr>
*/

#include "common.h"
#include "nut-scan.h"
#include "nut_stdint.h"

/* externally visible to nutscan-init */
int nutscan_unload_ipmi_library(void);

#ifdef WITH_IPMI

#include <freeipmi/freeipmi.h>
#include <string.h>
#include <stdio.h>
#include <ltdl.h>

#define NUT_IPMI_DRV_NAME	"nut-ipmipsu"

/* IPMI defines */
/* 5 seconds for establishing an IPMI connection */
#define IPMI_SESSION_TIMEOUT_LENGTH_DEFAULT			5000
#define IPMI_RETRANSMISSION_TIMEOUT_LENGTH_DEFAULT	250

/* dynamic link library stuff */
static lt_dlhandle dl_handle = NULL;
static const char *dl_error = NULL;
static char *dl_saved_libname = NULL;

#ifdef HAVE_FREEIPMI_11X_12X
  /* Functions symbols remapping */
#  define IPMI_FRU_CLOSE_DEVICE_ID                     "ipmi_fru_close_device_id"
#  define IPMI_FRU_CTX_DESTROY                         "ipmi_fru_ctx_destroy"
#  define IPMI_FRU_CTX_CREATE                          "ipmi_fru_ctx_create"
#  define IPMI_FRU_CTX_SET_FLAGS                       "ipmi_fru_ctx_set_flags"
#  define IPMI_FRU_OPEN_DEVICE_ID                      "ipmi_fru_open_device_id"
#  define IPMI_FRU_CTX_ERRORMSG                        "ipmi_fru_ctx_errormsg"
#  define IPMI_FRU_READ_DATA_AREA                      "ipmi_fru_read_data_area"
#  define IPMI_FRU_PARSE_NEXT                          "ipmi_fru_next"
  typedef ipmi_fru_ctx_t ipmi_fru_parse_ctx_t;
  typedef ipmi_sdr_ctx_t ipmi_sdr_cache_ctx_t;
  /* Functions remapping */
  static void (*nut_ipmi_sdr_ctx_destroy) (ipmi_sdr_ctx_t ctx);
#else /* HAVE_FREEIPMI_11X_12X */
#  define IPMI_FRU_AREA_SIZE_MAX                                   IPMI_FRU_PARSE_AREA_SIZE_MAX
#  define IPMI_FRU_FLAGS_SKIP_CHECKSUM_CHECKS                      IPMI_FRU_PARSE_FLAGS_SKIP_CHECKSUM_CHECKS
#  define IPMI_FRU_AREA_TYPE_MULTIRECORD_POWER_SUPPLY_INFORMATION  IPMI_FRU_PARSE_AREA_TYPE_MULTIRECORD_POWER_SUPPLY_INFORMATION
  /* Functions symbols remapping */
#  define IPMI_FRU_CLOSE_DEVICE_ID                     "ipmi_fru_parse_close_device_id"
#  define IPMI_FRU_CTX_DESTROY                         "ipmi_fru_parse_ctx_destroy"
#  define IPMI_FRU_CTX_CREATE                          "ipmi_fru_parse_ctx_create"
#  define IPMI_FRU_CTX_SET_FLAGS                       "ipmi_fru_parse_ctx_set_flags"
#  define IPMI_FRU_OPEN_DEVICE_ID                      "ipmi_fru_parse_open_device_id"
#  define IPMI_FRU_CTX_ERRORMSG                        "ipmi_fru_parse_ctx_errormsg"
#  define IPMI_FRU_READ_DATA_AREA                      "ipmi_fru_parse_read_data_area"
#  define IPMI_FRU_PARSE_NEXT                          "ipmi_fru_parse_next"
  /* Functions remapping */
  static void (*nut_ipmi_sdr_cache_ctx_destroy) (ipmi_sdr_cache_ctx_t ctx);
  static void (*nut_ipmi_sdr_parse_ctx_destroy) (ipmi_sdr_parse_ctx_t ctx);
#endif /* HAVE_FREEIPMI_11X_12X */


static int (*nut_ipmi_fru_close_device_id) (ipmi_fru_parse_ctx_t ctx);
static void (*nut_ipmi_fru_ctx_destroy) (ipmi_fru_parse_ctx_t ctx);
static ipmi_fru_parse_ctx_t (*nut_ipmi_fru_ctx_create) (ipmi_ctx_t ipmi_ctx);
static int (*nut_ipmi_fru_ctx_set_flags) (ipmi_fru_parse_ctx_t ctx, unsigned int flags);
static int (*nut_ipmi_fru_open_device_id) (ipmi_fru_parse_ctx_t ctx, uint8_t fru_device_id);
static char * (*nut_ipmi_fru_ctx_errormsg) (ipmi_fru_parse_ctx_t ctx);
static int (*nut_ipmi_fru_read_data_area) (ipmi_fru_parse_ctx_t ctx,
                                   unsigned int *area_type,
                                   unsigned int *area_length,
                                   void *areabuf,
                                   unsigned int areabuflen);
static int (*nut_ipmi_fru_next) (ipmi_fru_parse_ctx_t ctx);
static ipmi_ctx_t (*nut_ipmi_ctx_create) (void);
static int (*nut_ipmi_ctx_find_inband) (ipmi_ctx_t ctx,
                            ipmi_driver_type_t *driver_type,
                            int disable_auto_probe,
                            uint16_t driver_address,
                            uint8_t register_spacing,
                            const char *driver_device,
                            unsigned int workaround_flags,
                            unsigned int flags);
static int (*nut_ipmi_ctx_open_outofband) (ipmi_ctx_t ctx,
                            const char *hostname,
                            const char *username,
                            const char *password,
                            uint8_t authentication_type,
                            uint8_t privilege_level,
                            unsigned int session_timeout,
                            unsigned int retransmission_timeout,
                            unsigned int workaround_flags,
                            unsigned int flags);
static int (*nut_ipmi_ctx_errnum) (ipmi_ctx_t ctx);
static char * (*nut_ipmi_ctx_errormsg) (ipmi_ctx_t ctx);
static int (*nut_ipmi_ctx_close) (ipmi_ctx_t ctx);
static void (*nut_ipmi_ctx_destroy) (ipmi_ctx_t ctx);

/* This variable collects device(s) from a sequential or parallel scan,
 * is returned to caller, and cleared to allow subsequent independent scans */
static nutscan_device_t * dev_ret = NULL;
#ifdef HAVE_PTHREAD
static pthread_mutex_t dev_mutex;
#endif

/* use explicit booleans */
#ifndef FALSE
typedef enum ebool { FALSE = 0, TRUE } bool_t;
#else
typedef int bool_t;
#endif

/* Internal functions */
static nutscan_device_t * nutscan_scan_ipmi_device(const char * IPaddr, nutscan_ipmi_t * sec);
static void * nutscan_scan_ipmi_device_thready(void * arg_sec);

/* Return 0 on success, -1 on error e.g. "was not loaded";
 * other values may be possible if lt_dlclose() errors set them;
 * visible externally */
int nutscan_unload_library(int *avail, lt_dlhandle *pdl_handle, char **libpath);
int nutscan_unload_ipmi_library(void)
{
	return nutscan_unload_library(&nutscan_avail_ipmi, &dl_handle, &dl_saved_libname);
}

/* Return 0 on error; visible externally */
int nutscan_load_ipmi_library(const char *libname_path);
int nutscan_load_ipmi_library(const char *libname_path)
{
	if (dl_handle != NULL) {
		/* if previous init failed */
		if (dl_handle == (void *)1) {
			return 0;
		}
		/* init has already been done */
		return 1;
	}

	if (libname_path == NULL) {
		upsdebugx(0, "IPMI library not found. IPMI search disabled.");
		return 0;
	}

	if (lt_dlinit() != 0) {
		upsdebugx(0, "%s: Error initializing lt_dlinit", __func__);
		return 0;
	}

	dl_handle = lt_dlopen(libname_path);
	if (!dl_handle) {
		dl_error = lt_dlerror();
		goto err;
	}

	/* Clear any existing error */
	lt_dlerror();

	*(void **) (&nut_ipmi_fru_close_device_id) = lt_dlsym(dl_handle, IPMI_FRU_CLOSE_DEVICE_ID);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_ctx_destroy) = lt_dlsym(dl_handle, IPMI_FRU_CTX_DESTROY);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

#ifdef HAVE_FREEIPMI_11X_12X

	*(void **) (&nut_ipmi_sdr_ctx_destroy) = lt_dlsym(dl_handle, "ipmi_sdr_ctx_destroy");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

#else /* HAVE_FREEIPMI_11X_12X */

	*(void **) (&nut_ipmi_sdr_cache_ctx_destroy) = lt_dlsym(dl_handle, "ipmi_sdr_cache_ctx_destroy");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_sdr_parse_ctx_destroy) = lt_dlsym(dl_handle, "ipmi_sdr_parse_ctx_destroy");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}
#endif /* HAVE_FREEIPMI_11X_12X */

	*(void **) (&nut_ipmi_fru_ctx_create) = lt_dlsym(dl_handle, IPMI_FRU_CTX_CREATE);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_ctx_set_flags) = lt_dlsym(dl_handle, IPMI_FRU_CTX_SET_FLAGS);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_open_device_id) = lt_dlsym(dl_handle, IPMI_FRU_OPEN_DEVICE_ID);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_ctx_errormsg) = lt_dlsym(dl_handle, IPMI_FRU_CTX_ERRORMSG);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_read_data_area) = lt_dlsym(dl_handle, IPMI_FRU_READ_DATA_AREA);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_fru_next) = lt_dlsym(dl_handle, IPMI_FRU_PARSE_NEXT);
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_create) = lt_dlsym(dl_handle, "ipmi_ctx_create");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_find_inband) = lt_dlsym(dl_handle, "ipmi_ctx_find_inband");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_open_outofband) = lt_dlsym(dl_handle, "ipmi_ctx_open_outofband");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_errnum) = lt_dlsym(dl_handle, "ipmi_ctx_errnum");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_errormsg) = lt_dlsym(dl_handle, "ipmi_ctx_errormsg");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_close) = lt_dlsym(dl_handle, "ipmi_ctx_close");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	*(void **) (&nut_ipmi_ctx_destroy) = lt_dlsym(dl_handle, "ipmi_ctx_destroy");
	if ((dl_error = lt_dlerror()) != NULL) {
			goto err;
	}

	if (dl_saved_libname)
		free(dl_saved_libname);
	dl_saved_libname = xstrdup(libname_path);

	return 1;

err:
	upsdebugx(0,
		"Cannot load IPMI library (%s) : %s. IPMI search disabled.",
		libname_path, dl_error);
	dl_handle = (void *)1;
	lt_dlexit();
	if (dl_saved_libname) {
		free(dl_saved_libname);
		dl_saved_libname = NULL;
	}
	return 0;
}
/* end of dynamic link library stuff */

/* Cleanup IPMI contexts */
#ifdef HAVE_FREEIPMI_11X_12X
static void nut_freeipmi_cleanup(ipmi_fru_parse_ctx_t fru_parse_ctx,
								 ipmi_sdr_ctx_t sdr_ctx)
#else /* HAVE_FREEIPMI_11X_12X */
static void nut_freeipmi_cleanup(ipmi_fru_parse_ctx_t fru_parse_ctx,
								 ipmi_sdr_cache_ctx_t sdr_cache_ctx,
								 ipmi_sdr_parse_ctx_t sdr_parse_ctx)
#endif /* HAVE_FREEIPMI_11X_12X */
{
	if (fru_parse_ctx) {
		(*nut_ipmi_fru_close_device_id) (fru_parse_ctx);
		(*nut_ipmi_fru_ctx_destroy) (fru_parse_ctx);
	}

#ifdef HAVE_FREEIPMI_11X_12X

	if (sdr_ctx) {
		(*nut_ipmi_sdr_ctx_destroy) (sdr_ctx);
	}

#else /* HAVE_FREEIPMI_11X_12X */

	if (sdr_cache_ctx) {
		(*nut_ipmi_sdr_cache_ctx_destroy) (sdr_cache_ctx);
	}

	if (sdr_parse_ctx) {
		(*nut_ipmi_sdr_parse_ctx_destroy) (sdr_parse_ctx);
	}

#endif /* HAVE_FREEIPMI_11X_12X */
}

/* Return 1 if supported, 0 otherwise */
static int is_ipmi_device_supported(ipmi_ctx_t ipmi_ctx, int ipmi_id)
{
	int ret = -1;
	unsigned int area_type = 0;
	unsigned int area_length = 0;
	uint8_t areabuf[IPMI_FRU_AREA_SIZE_MAX + 1];
	ipmi_fru_parse_ctx_t fru_parse_ctx = NULL;
#ifdef HAVE_FREEIPMI_11X_12X
	ipmi_sdr_ctx_t sdr_ctx = NULL;
#else /* HAVE_FREEIPMI_11X_12X */
	ipmi_sdr_cache_ctx_t sdr_cache_ctx = NULL;
	ipmi_sdr_parse_ctx_t sdr_parse_ctx = NULL;
#endif /* HAVE_FREEIPMI_11X_12X */

	/* Parse FRU information */
	if (!(fru_parse_ctx = (*nut_ipmi_fru_ctx_create) (ipmi_ctx)))
	{
		upsdebugx(0, "%s: Error with %s(): %s",
			__func__, IPMI_FRU_CTX_CREATE,
			(*nut_ipmi_ctx_errormsg)(ipmi_ctx));
		return 0;
	}

	/* lots of motherboards calculate checksums incorrectly */
	if ((*nut_ipmi_fru_ctx_set_flags) (fru_parse_ctx, IPMI_FRU_FLAGS_SKIP_CHECKSUM_CHECKS) < 0)
	{
#ifdef HAVE_FREEIPMI_11X_12X
		nut_freeipmi_cleanup(fru_parse_ctx, sdr_ctx);
#else
		nut_freeipmi_cleanup(fru_parse_ctx, sdr_cache_ctx, sdr_parse_ctx);
#endif /* HAVE_FREEIPMI_11X_12X */
		return 0;
	}

	if (ipmi_id < 0 || (unsigned int)ipmi_id > UINT8_MAX) {
		upsdebugx(0, "%s: ipmi_id=%d is out of range!",
			__func__, ipmi_id);
		return 0;
	}
	if ((*nut_ipmi_fru_open_device_id) (fru_parse_ctx, (uint8_t)ipmi_id) < 0)
	{
#ifdef HAVE_FREEIPMI_11X_12X
		nut_freeipmi_cleanup(fru_parse_ctx, sdr_ctx);
#else
		nut_freeipmi_cleanup(fru_parse_ctx, sdr_cache_ctx, sdr_parse_ctx);
#endif /* HAVE_FREEIPMI_11X_12X */
		return 0;
	}

	do
	{
		/* clear fields */
		area_type = 0;
		area_length = 0;
		memset (areabuf, '\0', IPMI_FRU_AREA_SIZE_MAX + 1);

		/* parse FRU buffer */
		if ((*nut_ipmi_fru_read_data_area) (fru_parse_ctx,
											&area_type,
											&area_length,
											areabuf,
											IPMI_FRU_AREA_SIZE_MAX) < 0)
		{
#ifdef HAVE_FREEIPMI_11X_12X
			nut_freeipmi_cleanup(fru_parse_ctx, sdr_ctx);
#else
			nut_freeipmi_cleanup(fru_parse_ctx, sdr_cache_ctx, sdr_parse_ctx);
#endif /* HAVE_FREEIPMI_11X_12X */
			return 0;
		}

		if (area_length)
		{
			if (area_type == IPMI_FRU_AREA_TYPE_MULTIRECORD_POWER_SUPPLY_INFORMATION)
			{
				/* Found a POWER_SUPPLY record */
#ifdef HAVE_FREEIPMI_11X_12X
				nut_freeipmi_cleanup(fru_parse_ctx, sdr_ctx);
#else
				nut_freeipmi_cleanup(fru_parse_ctx, sdr_cache_ctx, sdr_parse_ctx);
#endif /* HAVE_FREEIPMI_11X_12X */
				return 1;
			}
		}
	} while ((ret = (*nut_ipmi_fru_next) (fru_parse_ctx)) == 1);

	/* No need for further errors checking */
#ifdef HAVE_FREEIPMI_11X_12X
	nut_freeipmi_cleanup(fru_parse_ctx, sdr_ctx);
#else
	nut_freeipmi_cleanup(fru_parse_ctx, sdr_cache_ctx, sdr_parse_ctx);
#endif /* HAVE_FREEIPMI_11X_12X */
	return 0;
}

static ipmi_ctx_t wrap_nut_ipmi_ctx_create(void)
{
	return (*nut_ipmi_ctx_create) ();
}

/* Check for IPMI support on a specific (local or remote) system
 * Return NULL on error, or a valid nutscan_device_t otherwise */
nutscan_device_t * nutscan_scan_ipmi_device(const char * IPaddr, nutscan_ipmi_t * ipmi_sec)
{
	ipmi_ctx_t ipmi_ctx = NULL;
	nutscan_device_t * nut_dev = NULL;
	nutscan_device_t * current_nut_dev = NULL;
	int ret = -1;
	int ipmi_id = 0;
	char port_id[64];

	if (!nutscan_avail_ipmi) {
		return NULL;
	}

	/* Initialize the FreeIPMI library. */
	if (!(ipmi_ctx = wrap_nut_ipmi_ctx_create()))
	{
		/* we have to force cleanup, since exit handler is not yet installed */
		upsdebugx(0, "%s: Failed to ipmi_ctx_create", __func__);
		return NULL;
	}

	/* Are we scanning locally, or over the network? */
	if (IPaddr == NULL)
	{
		upsdebugx(2, "Entering %s for local device scan", __func__);

		/* FIXME: we need root right to access local IPMI!
		if (!ipmi_is_root ()) {
			upsdebugx(0, "%s: IPMI scan: %s", __func__, ipmi_ctx_strerror (IPMI_ERR_PERMISSION));
		} */

		if ((ret = (*nut_ipmi_ctx_find_inband) (ipmi_ctx,
					NULL,
					0, /* don't disable auto-probe */
					0,
					0,
					NULL,
					0, /* workaround flags, none by default */
					0  /* flags */
					)) < 0)
		{
			upsdebugx(2, "ipmi_ctx_find_inband (local scan): %s",
				(*nut_ipmi_ctx_errormsg) (ipmi_ctx));
			return NULL;
		}
		if (!ret)
		{
			/* No local IPMI device detected */
			return NULL;
		}
	}
	else {

		upsdebugx(2, "Entering %s for %s", __func__, IPaddr);

#if 0
		if (ipmi_sec->ipmi_version == IPMI_2_0) {

			/* FIXME: need processing?!
			 * int parse_kg (void *out, unsigned int outlen, const char *in)
			 * if ((rv = parse_kg (common_cmd_args_config->k_g, IPMI_MAX_K_G_LENGTH + 1, data->string)) < 0)
			 * {
			 * 	upsdebugx(0, "%s: Config File Error: k_g input formatted incorrectly", __func__);
			 * 	exit (EXIT_FAILURE);
			 * }*/
			if ((ret = (*nut_ipmi_ctx_open_outofband_2_0) (ipmi_ctx,
															IPaddr,
															ipmi_sec->username,
															ipmi_sec->password,
															ipmi_sec->K_g_BMC_key,
/*???*/														(ipmi_sec->K_g_BMC_key) ? config->k_g_len : 0,
															ipmi_sec->privilege_level,
															ipmi_sec->cipher_suite_id,
															IPMI_SESSION_TIMEOUT_LENGTH_DEFAULT,
															IPMI_RETRANSMISSION_TIMEOUT_LENGTH_DEFAULT,
															ipmi_dev->workaround_flags,
															flags) < 0)
			{
				upsdebugx(2, "nut_ipmi_ctx_open_outofband_2_0 (%s): %s",
					IPaddr, (*nut_ipmi_ctx_errormsg) (c->ipmi_ctx));
				IPMI_MONITORING_DEBUG (("ipmi_ctx_open_outofband_2_0 (%s): %s",
					IPaddr, ipmi_ctx_errormsg (c->ipmi_ctx)));

				if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_USERNAME_INVALID)
					c->errnum = IPMI_MONITORING_ERR_USERNAME_INVALID;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_PASSWORD_INVALID)
					c->errnum = IPMI_MONITORING_ERR_PASSWORD_INVALID;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_PRIVILEGE_LEVEL_INSUFFICIENT)
					c->errnum = IPMI_MONITORING_ERR_PRIVILEGE_LEVEL_INSUFFICIENT;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_PRIVILEGE_LEVEL_CANNOT_BE_OBTAINED)
					c->errnum = IPMI_MONITORING_ERR_PRIVILEGEL_LEVEL_CANNOT_BE_OBTAINED;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_K_G_INVALID)
					c->errnum = IPMI_MONITORING_ERR_K_G_INVALID;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_CIPHER_SUITE_ID_UNAVAILABLE)
					c->errnum = IPMI_MONITORING_ERR_CIPHER_SUITE_ID_UNAVAILABLE;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_PASSWORD_VERIFICATION_TIMEOUT)
					c->errnum = IPMI_MONITORING_ERR_PASSWORD_VERIFICATION_TIMEOUT;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_IPMI_2_0_UNAVAILABLE)
					c->errnum = IPMI_MONITORING_ERR_IPMI_2_0_UNAVAILABLE;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_CONNECTION_TIMEOUT)
					c->errnum = IPMI_MONITORING_ERR_CONNECTION_TIMEOUT;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_SESSION_TIMEOUT)
					c->errnum = IPMI_MONITORING_ERR_SESSION_TIMEOUT;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_BAD_COMPLETION_CODE
					   || ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_IPMI_ERROR)
					c->errnum = IPMI_MONITORING_ERR_IPMI_ERROR;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_BMC_BUSY)
					c->errnum = IPMI_MONITORING_ERR_BMC_BUSY;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_OUT_OF_MEMORY)
					c->errnum = IPMI_MONITORING_ERR_OUT_OF_MEMORY;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_HOSTNAME_INVALID)
					c->errnum = IPMI_MONITORING_ERR_HOSTNAME_INVALID;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_PARAMETERS)
					c->errnum = IPMI_MONITORING_ERR_PARAMETERS;
				else if (ipmi_ctx_errnum (c->ipmi_ctx) == IPMI_ERR_SYSTEM_ERROR)
					c->errnum = IPMI_MONITORING_ERR_SYSTEM_ERROR;
				else
					c->errnum = IPMI_MONITORING_ERR_INTERNAL_ERROR;
				return (-1);
			}
		}
		else { /* Not IPMI 2.0 */

#endif /* 0 */

		/* Fall back to IPMI 1.5 */
		if (ipmi_sec->authentication_type < 0
		    || (unsigned int)ipmi_sec->authentication_type > UINT8_MAX
		) {
			upsdebugx(2, "nutscan_scan_ipmi_device (%s): "
				"authentication_type=%d is out of range!",
				IPaddr, ipmi_sec->authentication_type);
			return 0;
		}
		if (ipmi_sec->privilege_level < 0
		    || (unsigned int)ipmi_sec->privilege_level > UINT8_MAX
		) {
			upsdebugx(2, "nutscan_scan_ipmi_device (%s): "
				"privilege_level=%d is out of range!",
				IPaddr, ipmi_sec->privilege_level);
			return 0;
		}
		if ((ret = (*nut_ipmi_ctx_open_outofband) (ipmi_ctx,
						IPaddr,
						ipmi_sec->username,
						ipmi_sec->password,
						(uint8_t)ipmi_sec->authentication_type,
						(uint8_t)ipmi_sec->privilege_level,
						IPMI_SESSION_TIMEOUT_LENGTH_DEFAULT,
						IPMI_RETRANSMISSION_TIMEOUT_LENGTH_DEFAULT,
						ipmi_sec->workaround_flags,
						IPMI_FLAGS_DEFAULT
						)) < 0)
		{
			/* No IPMI device detected on this host!
			if ((*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_USERNAME_INVALID
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_PASSWORD_INVALID
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_PRIVILEGE_LEVEL_INSUFFICIENT
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_PRIVILEGE_LEVEL_CANNOT_BE_OBTAINED
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_AUTHENTICATION_TYPE_UNAVAILABLE
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_PASSWORD_VERIFICATION_TIMEOUT
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_HOSTNAME_INVALID
			  || (*nut_ipmi_ctx_errnum) (ipmi_ctx) == IPMI_ERR_CONNECTION_TIMEOUT) { */

				/* FIXME: don't log timeout errors */
				upsdebugx(2, "nut_ipmi_ctx_open_outofband (%s): %s",
					IPaddr, (*nut_ipmi_ctx_errormsg) (ipmi_ctx));
				return NULL;
			/*}*/
		}
	}

	/* Loop through all possible components */
	for (ipmi_id = 0 ; ipmi_id <= IPMI_FRU_DEVICE_ID_MAX ; ipmi_id++) {

		if (is_ipmi_device_supported(ipmi_ctx, ipmi_id)) {

			if ((nut_dev = nutscan_new_device()) == NULL) {
				upsdebugx(0, "%s: Memory allocation error", __func__);
				nutscan_free_device(current_nut_dev);
				break;
			}

			/* Fill the device structure (sufficient with driver and port) */
			nut_dev->type = TYPE_IPMI;
			nut_dev->driver = strdup(NUT_IPMI_DRV_NAME);
			if (IPaddr == NULL) {
				sprintf(port_id, "id%x", (unsigned int)ipmi_id);
			}
			else {
				/* FIXME: also check against "localhost" and its IPv{4,6} */
				/* FIXME: Should the IPv6 address here be bracketed?
				 *  Does our driver support the notation? */
				sprintf(port_id, "id%x@%s", (unsigned int)ipmi_id, IPaddr);
			}
			nut_dev->port = strdup(port_id);
			/* FIXME: also dump device.serial?
			 * using drivers/libfreeipmi_get_board_info() */

			current_nut_dev = nutscan_add_device_to_device(
							current_nut_dev,
							nut_dev);

			memset (port_id, 0, sizeof(port_id));
		}
	}

	/* Final cleanup */
	if (ipmi_ctx) {
		(*nut_ipmi_ctx_close) (ipmi_ctx);
		(*nut_ipmi_ctx_destroy) (ipmi_ctx);
	}

	return current_nut_dev;
}

/* Wrap calls to nutscan_scan_ipmi_device() into semantics for parallel-able
 * scanning. Returns NULL, updates global dev_ret when a scan is successful.
 * FREES the caller's copy of "sec" and "peername" in it, if applicable.
 */
static void * nutscan_scan_ipmi_device_thready(void * arg_sec)
{
	nutscan_device_t	*dev = NULL;
	nutscan_ipmi_t	*sec = (nutscan_ipmi_t *)arg_sec;

	if (sec == NULL || sec->peername == NULL)
		dev = nutscan_scan_ipmi_device(NULL, NULL);
	else
		dev = nutscan_scan_ipmi_device(sec->peername, sec);

	if (dev) {
#ifdef HAVE_PTHREAD
		pthread_mutex_lock(&dev_mutex);
#endif
		dev_ret = nutscan_add_device_to_device(dev_ret, dev);
#ifdef HAVE_PTHREAD
		pthread_mutex_unlock(&dev_mutex);
#endif
	}

	if (sec) {
		if (sec->peername) {
			free(sec->peername);
		}
		free(sec);
	}

	return NULL;
}

/* General IPMI scan entry point: scan 1 to n devices, local or remote,
 * for IPMI support
 * Return NULL on error, or a valid nutscan_device_t otherwise */
nutscan_device_t * nutscan_scan_ipmi(const char * start_ip, const char * stop_ip, nutscan_ipmi_t * sec)
{
	nutscan_device_t	*ndret;

	/* Are we scanning locally, or through the network? */
	if (start_ip || stop_ip) {
		nutscan_ip_range_list_t irl;

		nutscan_init_ip_ranges(&irl);
		nutscan_add_ip_range(&irl, (char *)start_ip, (char *)stop_ip);

		ndret = nutscan_scan_ip_range_ipmi(&irl, sec);

		/* Avoid nuking caller's strings here */
		irl.ip_ranges->start_ip = NULL;
		irl.ip_ranges->end_ip = NULL;
		nutscan_free_ip_ranges(&irl);
	} else {
		/* Probe local device */
		ndret = nutscan_scan_ip_range_ipmi(NULL, sec);
	}

	return ndret;
}

/* General IPMI scan entry point: scan 1 to n devices, local or remote,
 * for IPMI support
 * Return NULL on error, or a valid nutscan_device_t otherwise */
nutscan_device_t * nutscan_scan_ip_range_ipmi(nutscan_ip_range_list_t * irl, nutscan_ipmi_t * sec)
{
	bool_t pass = TRUE; /* Track that we may spawn a scanning thread */
	nutscan_device_t * result = NULL;
	nutscan_ipmi_t * tmp_sec = NULL;

	if (!nutscan_avail_ipmi) {
		return NULL;
	}

	/* Are we scanning locally, or through the network?
	 * We assume the list is maintained by our methods, so should not have
	 * null addresses. But just in case - check for it a little tiny once.
	 */
	if (irl == NULL || irl->ip_ranges == NULL
	 || irl->ip_ranges->start_ip == NULL || irl->ip_ranges->end_ip == NULL
	) {
		upsdebugx(1, "%s: Local PSU scan", __func__);
		nutscan_scan_ipmi_device_thready(NULL);
	} else {
		/* Iterate the one or a range of IPs to scan */
		nutscan_ip_range_list_iter_t ip;
		char * ip_str = NULL;

#ifdef HAVE_PTHREAD
# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
		sem_t * semaphore = nutscan_semaphore();
#  if (defined HAVE_SEMAPHORE_UNNAMED)
		sem_t   semaphore_scantype_inst;
		sem_t * semaphore_scantype = &semaphore_scantype_inst;
#  elif (defined HAVE_SEMAPHORE_NAMED)
		sem_t * semaphore_scantype = NULL;
#  endif
# endif /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */
		pthread_t thread;
		nutscan_thread_t * thread_array = NULL;
		size_t thread_count = 0, i;
# if (defined HAVE_PTHREAD_TRYJOIN) || (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
		size_t  max_threads_scantype = max_threads_ipmi;
# endif
#endif	/* HAVE_PTHREAD */

		if (irl->ip_ranges_count == 1
		&& (irl->ip_ranges->start_ip == irl->ip_ranges->end_ip
		    || !strcmp(irl->ip_ranges->start_ip, irl->ip_ranges->end_ip)
		)) {
			upsdebugx(1, "%s: Scanning remote PSU for single IP address: %s",
				__func__, irl->ip_ranges->start_ip);
		} else {
			upsdebugx(1, "%s: Scanning remote PSU for IP address range(s): %s",
				__func__, nutscan_stringify_ip_ranges(irl));
		}

#ifdef HAVE_PTHREAD
		pthread_mutex_init(&dev_mutex, NULL);

# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
		if (max_threads_scantype > 0) {
#ifdef HAVE_PRAGMAS_FOR_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE
#pragma GCC diagnostic push
#endif
#ifdef HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE
#pragma GCC diagnostic ignored "-Wunreachable-code"
#endif
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunreachable-code"
#endif
			/* Different platforms, different sizes, none fits all... */
			if (SIZE_MAX > UINT_MAX && max_threads_scantype > UINT_MAX) {
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#ifdef HAVE_PRAGMAS_FOR_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE
#pragma GCC diagnostic pop
#endif
				upsdebugx(1,
					"WARNING: %s: Limiting max_threads_scantype to range acceptable for " REPORT_SEM_INIT_METHOD "()",
					__func__);
				max_threads_scantype = UINT_MAX - 1;
			}

			upsdebugx(4, "%s: " REPORT_SEM_INIT_METHOD "() for %" PRIuSIZE " threads", __func__, max_threads_scantype);
#  if (defined HAVE_SEMAPHORE_UNNAMED)
			if (sem_init(semaphore_scantype, 0, (unsigned int)max_threads_scantype)) {
				upsdebug_with_errno(4, "%s: " REPORT_SEM_INIT_METHOD "() failed", __func__);
				max_threads_scantype = 0;
			}
#  elif (defined HAVE_SEMAPHORE_NAMED)
			if (SEM_FAILED == (semaphore_scantype = sem_open(SEMNAME_IPMI, O_CREAT, 0644, (unsigned int)max_threads_scantype))) {
				upsdebug_with_errno(4, "%s: " REPORT_SEM_INIT_METHOD "() failed", __func__);
				semaphore_scantype = NULL;
				max_threads_scantype = 0;
			}
#  endif
		}
# endif /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */

#endif /* HAVE_PTHREAD */

		ip_str = nutscan_ip_ranges_iter_init(&ip, irl);

		while (ip_str != NULL) {
#ifdef HAVE_PTHREAD
			/* NOTE: With many enough targets to scan, this can crash
			 * by spawning too many children; add a limit and loop to
			 * "reap" some already done with their work. And probably
			 * account them in thread_array[] as something to not wait
			 * for below in pthread_join()...
			 */

# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
			/* Just wait for someone to free a semaphored slot,
			 * if none are available, and then/otherwise grab one
			 */
			if (thread_array == NULL) {
				/* Starting point, or after a wait to complete
				 * all earlier runners */
				if (max_threads_scantype > 0)
					sem_wait(semaphore_scantype);
				sem_wait(semaphore);
				pass = TRUE;
			} else {
				/* If successful (the lock was acquired),
				 * sem_wait() and sem_trywait() will return 0.
				 * Otherwise, -1 is returned and errno is set,
				 * and the state of the semaphore is unchanged.
				 */
				int	stwST = sem_trywait(semaphore_scantype);
				int	stwS  = sem_trywait(semaphore);
				pass = ((max_threads_scantype == 0 || stwST == 0) && stwS == 0);
				upsdebugx(4, "%s: max_threads_scantype=%" PRIuSIZE
					" curr_threads=%" PRIuSIZE
					" thread_count=%" PRIuSIZE
					" stwST=%d stwS=%d pass=%u",
					__func__, max_threads_scantype,
					curr_threads, thread_count,
					stwST, stwS, pass
				);
			}
# else
#  ifdef HAVE_PTHREAD_TRYJOIN
			/* A somewhat naive and brute-force solution for
			 * systems without a semaphore.h. This may suffer
			 * some off-by-one errors, using a few more threads
			 * than intended (if we race a bit at the wrong time,
			 * probably up to one per enabled scanner routine).
			 */

			/* TOTHINK: Should there be a threadcount_mutex when
			 * we just read the value in if() and while() below?
			 * At worst we would overflow the limit a bit due to
			 * other protocol scanners...
			 */
			if (curr_threads >= max_threads
			|| (curr_threads >= max_threads_scantype && max_threads_scantype > 0)
			) {
				upsdebugx(2, "%s: already running %" PRIuSIZE " scanning threads "
					"(launched overall: %" PRIuSIZE "), "
					"waiting until some would finish",
					__func__, curr_threads, thread_count);

				while (curr_threads >= max_threads
				   || (curr_threads >= max_threads_scantype && max_threads_scantype > 0)
				) {
					for (i = 0; i < thread_count ; i++) {
						int ret;

						if (!thread_array[i].active) continue;

						pthread_mutex_lock(&threadcount_mutex);
						upsdebugx(3, "%s: Trying to join thread #%" PRIuSIZE "...", __func__, i);
						ret = pthread_tryjoin_np(thread_array[i].thread, NULL);
						switch (ret) {
							case ESRCH:     /* No thread with the ID thread could be found - already "joined"? */
								upsdebugx(5, "%s: Was thread #%" PRIuSIZE " joined earlier?", __func__, i);
								break;
							case 0:         /* thread exited */
								if (curr_threads > 0) {
									curr_threads --;
									upsdebugx(4, "%s: Joined a finished thread #%" PRIuSIZE, __func__, i);
								} else {
									/* threadcount_mutex fault? */
									upsdebugx(0, "WARNING: %s: Accounting of thread count "
										"says we are already at 0", __func__);
								}
								thread_array[i].active = FALSE;
								break;
							case EBUSY:     /* actively running */
								upsdebugx(6, "%s: thread #%" PRIuSIZE " still busy (%i)",
									__func__, i, ret);
								break;
							case EDEADLK:   /* Errors with thread interactions... bail out? */
							case EINVAL:    /* Errors with thread interactions... bail out? */
							default:        /* new pthreads abilities? */
								upsdebugx(5, "%s: thread #%" PRIuSIZE " reported code %i",
									__func__, i, ret);
								break;
						}
						pthread_mutex_unlock(&threadcount_mutex);
					}

					if (curr_threads >= max_threads
					|| (curr_threads >= max_threads_scantype && max_threads_scantype > 0)
					) {
							usleep (10000); /* microSec's, so 0.01s here */
					}
				}
				upsdebugx(2, "%s: proceeding with scan", __func__);
			}

			/* NOTE: No change to default "pass" in this ifdef:
			 * if we got to this line, we have a slot to use */
#  endif /* HAVE_PTHREAD_TRYJOIN */
# endif  /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */
#endif   /* HAVE_PTHREAD */

			if (pass) {
				tmp_sec = malloc(sizeof(nutscan_ipmi_t));
				if (tmp_sec == NULL) {
					upsdebugx(0, "%s: Memory allocation error", __func__);
					break;
				}

				memcpy(tmp_sec, sec, sizeof(nutscan_ipmi_t));
				tmp_sec->peername = ip_str;

#ifdef HAVE_PTHREAD
				if (pthread_create(&thread, NULL, nutscan_scan_ipmi_device_thready, (void*)tmp_sec) == 0) {
					nutscan_thread_t	*new_thread_array;
# ifdef HAVE_PTHREAD_TRYJOIN
					pthread_mutex_lock(&threadcount_mutex);
					curr_threads++;
# endif /* HAVE_PTHREAD_TRYJOIN */

					thread_count++;
					new_thread_array = realloc(thread_array,
						thread_count * sizeof(nutscan_thread_t));
					if (new_thread_array == NULL) {
						upsdebugx(1, "%s: Failed to realloc thread array", __func__);
						break;
					}
					else {
						thread_array = new_thread_array;
					}
					thread_array[thread_count - 1].thread = thread;
					thread_array[thread_count - 1].active = TRUE;

# ifdef HAVE_PTHREAD_TRYJOIN
					pthread_mutex_unlock(&threadcount_mutex);
# endif /* HAVE_PTHREAD_TRYJOIN */
				}
#else	/* if not HAVE_PTHREAD */
				nutscan_scan_ipmi_device_thready(tmp_sec);
#endif	/* if HAVE_PTHREAD */

				/* Prepare the next iteration; note that
				 * nutscan_scan_ipmi_device_thready()
				 * takes care of freeing "tmp_sec" and its
				 * reference (NOT strdup!) to "ip_str" as
				 * peername.
				 */
				ip_str = nutscan_ip_ranges_iter_inc(&ip);
			} else { /* if not pass -- all slots busy */
#ifdef HAVE_PTHREAD
# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
				/* Wait for all current scans to complete */
				if (thread_array != NULL) {
					upsdebugx (2, "%s: Running too many scanning threads (%"
						PRIuSIZE "), "
						"waiting until older ones would finish",
						__func__, thread_count);
					for (i = 0; i < thread_count ; i++) {
						int ret;
						if (!thread_array[i].active) {
							/* Probably should not get here,
							 * but handle it just in case */
							upsdebugx(0, "WARNING: %s: Midway clean-up: did not expect thread %" PRIuSIZE " to be not active",
								__func__, i);
							sem_post(semaphore);
							if (max_threads_scantype > 0)
								sem_post(semaphore_scantype);
							continue;
						}
						thread_array[i].active = FALSE;
						ret = pthread_join(thread_array[i].thread, NULL);
						if (ret != 0) {
							upsdebugx(0, "WARNING: %s: Midway clean-up: pthread_join() returned code %i",
								__func__, ret);
						}
						sem_post(semaphore);
						if (max_threads_scantype > 0)
							sem_post(semaphore_scantype);
					}
					thread_count = 0;
					free(thread_array);
					thread_array = NULL;
				}
# else
#  ifdef HAVE_PTHREAD_TRYJOIN
				/* TODO: Move the wait-loop for TRYJOIN here? */
#  endif /* HAVE_PTHREAD_TRYJOIN */
# endif  /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */
#endif   /* HAVE_PTHREAD */
			} /* if: could we "pass" or not? */
		} /* while */

#ifdef HAVE_PTHREAD
		if (thread_array != NULL) {
			upsdebugx(2, "%s: all planned scans launched, waiting for threads to complete", __func__);
			for (i = 0; i < thread_count; i++) {
				int ret;

				if (!thread_array[i].active) continue;

				ret = pthread_join(thread_array[i].thread, NULL);
				if (ret != 0) {
					upsdebugx(0, "WARNING: %s: Clean-up: pthread_join() returned code %i",
						__func__, ret);
				}
				thread_array[i].active = FALSE;
# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
				sem_post(semaphore);
				if (max_threads_scantype > 0)
					sem_post(semaphore_scantype);
# else
#  ifdef HAVE_PTHREAD_TRYJOIN
				pthread_mutex_lock(&threadcount_mutex);
				if (curr_threads > 0) {
					curr_threads --;
					upsdebugx(5, "%s: Clean-up: Joined a finished thread #%" PRIuSIZE,
						__func__, i);
				} else {
					upsdebugx(0, "WARNING: %s: Clean-up: Accounting of thread count "
						"says we are already at 0", __func__);
				}
				pthread_mutex_unlock(&threadcount_mutex);
#  endif /* HAVE_PTHREAD_TRYJOIN */
# endif /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */
			}
			free(thread_array);
			upsdebugx(2, "%s: all threads freed", __func__);
		}
		pthread_mutex_destroy(&dev_mutex);

# if (defined HAVE_SEMAPHORE_UNNAMED) || (defined HAVE_SEMAPHORE_NAMED)
		if (max_threads_scantype > 0) {
#  if (defined HAVE_SEMAPHORE_UNNAMED)
			sem_destroy(semaphore_scantype);
#  elif (defined HAVE_SEMAPHORE_NAMED)
			if (semaphore_scantype) {
				sem_unlink(SEMNAME_IPMI);
				sem_close(semaphore_scantype);
				semaphore_scantype = NULL;
			}
#  endif
		}
# endif /* HAVE_SEMAPHORE_UNNAMED || HAVE_SEMAPHORE_NAMED */
#endif /* HAVE_PTHREAD */
	}	/* end of: scan range of 1+ IP address(es), maybe in parallel */

	result = nutscan_rewind_device(dev_ret);
	dev_ret = NULL;
	return result;
}

#else /* not WITH_IPMI */

/* stub function */
nutscan_device_t *  nutscan_scan_ipmi(const char * startIP, const char * stopIP, nutscan_ipmi_t * sec)
{
	NUT_UNUSED_VARIABLE(startIP);
	NUT_UNUSED_VARIABLE(stopIP);
	NUT_UNUSED_VARIABLE(sec);

	return NULL;
}

/* stub function */
nutscan_device_t *  nutscan_scan_ip_range_ipmi(nutscan_ip_range_list_t * irl, nutscan_ipmi_t * sec)
{
	NUT_UNUSED_VARIABLE(irl);
	NUT_UNUSED_VARIABLE(sec);

	return NULL;
}

int nutscan_unload_ipmi_library(void)
{
	return 0;
}
#endif /* WITH_IPMI */
