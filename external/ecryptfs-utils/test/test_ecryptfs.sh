#!/system/bin/sh

# Copyright (C) 2013 Sony Mobile Communications AB.
# All rights, including trade secret rights, reserved.
#
# @author PLD/Security United

#
# Tests for eCryptfs mount and umount commands
#

# Error codes
OK=0
ERROR=1

# eCryptfs folders
DIR1=/data/ecryptfs_tmp/1
DIR2=/data/ecryptfs_tmp/2

# Mounting commands
MOUNT=mount.ecryptfs
UMOUNT=umount.ecryptfs

# Passphrases
PASS1="test1"
PASS2="test2"

# File data
FILE_DATA1="Filedata1"
FILE_DATA2="Filedata2"

# Filename
FILENAME="file.txt"
FILENAME_ENC_PASS1="ECRYPTFS_FNEK_ENCRYPTED.FWYRuyhRMPvwqkZlgCDr.nidVvoXrzb6hk.RQYkFT9tUqmTyCkqb8jxQU---"

function on_error() {
    ERR=$1
    MSG=$2
    if [ $ERR != $OK ] ; then
        echo "failed with code $ERR: $MSG"
        return $ERR
    fi
}

function create_tmp_dirs() {
    mkdir -p $DIR1
    mkdir -p $DIR2
}

function rm_tmp_dirs() {
    rm -rf $DIR1
    rm -rf $DIR2
}

# Supporting test functions
function mount_folder() {
    FOLDER=$1
    PASSWD=$2
    DEV_NULL=`$MOUNT -f $FOLDER -p $PASSWD`
}

function umount_folder() {
    FOLDER=$1
    DEV_NULL=`$UMOUNT -f $FOLDER`
}

function create_file_content() {
    NAME=$1
    CONTENT=$2
    echo ${CONTENT} > ${NAME}
    on_error $? "writing \"${CONTENT}\" to ${NAME}"
}

function check_file_content() {
    NAME=$1
    CONTENT=$2
    READ_CONTENT=$(cat ${NAME})
    if [ "$CONTENT" = "$READ_CONTENT" ] ; then
        return $OK
    else
        return $ERROR
    fi
}

# Mount and umount test case
function test_mount_and_umount() {
    mount_folder $DIR1 $PASS1
    on_error $? "mounting $DIR1 with password $PASS1"
    mount_folder $DIR2 $PASS1
    on_error $? "mounting $DIR2 with password $PASS1"

    umount_folder $DIR1
    on_error $? "unmounting $DIR1"
    umount_folder $DIR2
    on_error $? "unmounting $DIR2"
}

# File data encryption test case
function test_file_encryption() {
    # Test that a mounted folder can be read and written
    mount_folder $DIR1 $PASS1
    on_error $? "mounting $DIR1 with password $PASS1"

    rm ${DIR1}/* &> /dev/null
    create_file_content ${DIR1}/${FILENAME} ${FILE_DATA1}
    check_file_content ${DIR1}/${FILENAME} ${FILE_DATA1}
    on_error $? "failed : ${DIR1}/${FILENAME} did not contain data \"${CONTENT}\""

    umount_folder $DIR1
    on_error $? "unmounting $DIR1"

    # Test that files get encrypted
    check_file_content ${DIR1}/${FILENAME_ENC_PASS1} ${FILE_DATA1}
    if [ $? = $OK ] ; then
        echo "failed : ${DIR1}/${FILENAME} contained data that wasn't encrypted"
        return $ERR
    fi

    # Test that a folder can be remounted without corrupting file data
    mount_folder $DIR1 $PASS1
    on_error $? "remounting $DIR1 with password $PASS1"

    check_file_content ${DIR1}/${FILENAME} ${FILE_DATA1}
    on_error $? "failed : ${DIR1}/${FILENAME} did not contain data \"${CONTENT}\" after remount"

    umount_folder $DIR1
    on_error $? "unmounting $DIR1"

    # Test that a file's data cannot be read correctly if mounted with incorrect password
    mount_folder $DIR1 $PASS2
    on_error $? "mounting $DIR1 with password $PASS2"

    ls ${DIR1}/${FILENAME} &> /dev/null
    if [ $? = $OK ] ; then
        check_file_content ${DIR1}/${FILENAME} ${FILE_DATA1}
        if [ $? = $OK ] ; then
            echo "failed : ${DIR1}/${FILENAME} contained data that was encrypted with incorrect password"
            return $ERR
        fi
    fi

    umount_folder $DIR1
    on_error $? "unmounting $DIR1"
}

# File data encryption test case
function test_filename_encryption() {
    mount_folder $DIR1 $PASS1
    on_error $? "mounting $DIR1 with password $PASS1"

    rm ${DIR1}/* &> /dev/null
    create_file_content ${DIR1}/${FILENAME} ${FILE_DATA1}

    ls ${DIR1}/${FILENAME} &> /dev/null
    on_error $? "file ${DIR1}/${FILENAME} didn't exist"

    umount_folder $DIR1
    on_error $? "unmounting $DIR1"

    ls ${DIR1}/${FILENAME} &> /dev/null
    if [ $? = $OK ] ; then
        echo "failed : file ${DIR1}/${FILENAME} existed and its filename wasn't encrypted"
        return $ERR
    fi
}

# Set-up environment
echo "setup environment"
create_tmp_dirs

# Run tests
echo "run tests:"
echo "run test_mount_and_umount"
test_mount_and_umount
echo "run test_file_encryption"
test_file_encryption
echo "test_filename_encryption"
test_filename_encryption

rm_tmp_dirs
echo "done. all tests passed"
