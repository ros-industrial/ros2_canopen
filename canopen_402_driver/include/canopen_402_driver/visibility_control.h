//    Copyright 2023 Christoph Hellmann Santos
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
#ifndef CANOPEN_402_DRIVER__VISIBILITY_CONTROL_H_
#define CANOPEN_402_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CANOPEN_402_DRIVER_EXPORT __attribute__((dllexport))
#define CANOPEN_402_DRIVER_IMPORT __attribute__((dllimport))
#else
#define CANOPEN_402_DRIVER_EXPORT __declspec(dllexport)
#define CANOPEN_402_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef CANOPEN_402_DRIVER_BUILDING_LIBRARY
#define CANOPEN_402_DRIVER_PUBLIC CANOPEN_402_DRIVER_EXPORT
#else
#define CANOPEN_402_DRIVER_PUBLIC CANOPEN_402_DRIVER_IMPORT
#endif
#define CANOPEN_402_DRIVER_PUBLIC_TYPE CANOPEN_402_DRIVER_PUBLIC
#define CANOPEN_402_DRIVER_LOCAL
#else
#define CANOPEN_402_DRIVER_EXPORT __attribute__((visibility("default")))
#define CANOPEN_402_DRIVER_IMPORT
#if __GNUC__ >= 4
#define CANOPEN_402_DRIVER_PUBLIC __attribute__((visibility("default")))
#define CANOPEN_402_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define CANOPEN_402_DRIVER_PUBLIC
#define CANOPEN_402_DRIVER_LOCAL
#endif
#define CANOPEN_402_DRIVER_PUBLIC_TYPE
#endif

#endif  // CANOPEN_402_DRIVER__VISIBILITY_CONTROL_H_
