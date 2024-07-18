#!/bin/sh

# Copyright (C) 2024 by Jim Klimov <jimklimov+nut@gmail.com>

SCRIPTDIR="`dirname "$0"`"
SCRIPTDIR="`cd "$SCRIPTDIR" && pwd`"

LTEXEC=""
[ -n "${LIBTOOL-}" ] || LIBTOOL="`command -v libtool`" 2>/dev/null
[ -z "${LIBTOOL}"  ] || LTEXEC="${LIBTOOL} --mode=execute "

# TODO: Also wrap tool=callgrind e.g. via $0 symlink name?

exec ${LTEXEC} \
valgrind \
	--tool=memcheck --verbose \
	--leak-check=full --show-reachable=yes --error-exitcode=1 --show-error-list=yes \
	--trace-children=yes --track-fds=yes --show-leak-kinds=all --track-origins=yes \
	--suppressions="${SCRIPTDIR}/.valgrind.supp" \
	${VALGRIND_OPTIONS-} \
	"$@"
