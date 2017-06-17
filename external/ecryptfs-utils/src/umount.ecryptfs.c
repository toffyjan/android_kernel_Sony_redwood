/* umount.ecryptfs.c
 *
 * Copyright (C) 2013 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#define LOG_TAG "umount.ecryptfs"
#include "cutils/log.h"
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <unistd.h>
#include <errno.h>

#include "keyhandling.h"

#define SIGNATURE_TAG "ecryptfs_sig="
#define MOUNT_LINE_LEN 1024

static char mount_line[MOUNT_LINE_LEN];

static char* get_ecryptfs_signature(char *path)
{
    char *ret = NULL;
    int len = strnlen(path, MOUNT_LINE_LEN);

    FILE *mount = fopen("/proc/mounts", "r");
    if (mount == NULL) {
        return NULL;
    }

    /* find the corresponding ecryptfs mount line */
    while (!feof(mount)) {
        char *line = fgets(mount_line, MOUNT_LINE_LEN, mount);
        if (line == NULL) {
            break;
        }

        if (strncmp(mount_line, path, len) == 0 && mount_line[len] == ' ') {
            /* extract the signature */
            ret = strstr(mount_line, SIGNATURE_TAG);
            if (ret != NULL) {
                ret += strnlen(SIGNATURE_TAG, MOUNT_LINE_LEN);
                ret = strtok(ret, ", ");
            }
            break;
        }
    }

    fclose(mount);
    return ret;
}

static void usage()
{
    SLOGE("usage: umount.ecryptfs -f <folder> [-s <signature>]\n"
            " -f    the folder to unmount\n"
            " -s    signature of a key loaded in kernel keyring");
}

int main(int argc, char *argv[])
{
    long ret = 0;
    char *folder = NULL;
    char *auth_tok_sig = NULL;
    int c;

    /* check parameters */
    while ((c = getopt(argc, argv, "f:s:")) != -1) {
        switch (c) {
        case 'f':
            folder = optarg;
            break;
        case 's':
            auth_tok_sig = optarg;
            break;
        default:
            SLOGE("\n");
            usage();
            return 1;
        }
    }

    /* check parameters */
    if (folder == NULL) {
        usage();
        return 2;
    }

    /* remove trailing '/' */
    if (folder[strnlen(folder, MOUNT_LINE_LEN) - 1] == '/') {
        folder[strnlen(folder, MOUNT_LINE_LEN) - 1] = 0;
    }

    /* get signature */
    if (auth_tok_sig == NULL) {
        auth_tok_sig = get_ecryptfs_signature(folder);
    }

    if (auth_tok_sig == NULL) {
        SLOGE("failed finding the authentication token signature or no signature given");
        return 3;
    }

    /* find key */
    int rc = keyctl_search(KEY_SPEC_USER_KEYRING, "user", auth_tok_sig, 0);
    if (rc != -1) {
        /* unlink key */
        ret = keyctl_unlink(rc, KEY_SPEC_USER_KEYRING);
        if (ret == -1) {
            SLOGE("failed to unlink key in keyring");
            return 4;
        }
    }

    /* unmount the folder */
    ret = umount2(folder, 0);
    if (ret != 0) {
        SLOGE("failed to unmount folder %s (%s)", folder, strerror(errno));
        return 5;
    }

    return ret;
}
