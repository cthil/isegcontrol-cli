#
# NamedConstant.py - ISEG SHQ command line controller
#
# Copyright (c) xxxx-2015 Christophe Thil <christophe.thil@ziti.uni-heidelberg.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

class NamedConstant(object):
    string_value = None
    int_value = None

    def __init__(self, string_value, int_value):
        self.string_value = string_value
        self.int_value = int_value

    def __repr__(self):
        return self.string_value

    def __str__(self):
        return self.string_value

    def __eq__(self, other):
        return self.int_value == int(other)
