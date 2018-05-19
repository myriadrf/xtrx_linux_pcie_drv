cmake_minimum_required(VERSION 2.8)
project(xtrx-pcie-linux-driver NONE)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION 1)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "all")

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
    set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_NAME                "xtrx-pcie-linux-driver")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "XTRX PCIe kernel dkms driver")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-1)
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "linux-headers-generic (>= 4.4.0)")

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SET_DESTDIR               "")
set(CPACK_PACKAGING_INSTALL_PREFIX  "/usr")
set(CPACK_PACKAGE_CONTACT           "Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>")

#########################################################################
# DKMS intallation
#########################################################################
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/prerm;" )

set(DKMSVER "${LIBVER}-${MAINT_VERSION}")
install(FILES Makefile dkms.conf xtrx.c xtrx_defs.h  DESTINATION "src/xtrx-${DKMSVER}" COMPONENT xtrx-drv)
install(FILES xtrxll_regs.vh DESTINATION "src/xtrx-${DKMSVER}" COMPONENT xtrx-drv)

########################################################################
# Install udev rules
########################################################################
option(INSTALL_UDEV_RULES "Install udev rules for xtrx-drv" OFF)
if (INSTALL_UDEV_RULES)
    set(UDEV_RULES_PATH
        "/etc/udev/rules.d"
        CACHE STRING
        "Target directory for udev rule installation. Ensure you have permissions to write to this directory."
        )
    install (
        FILES 50-xtrx.rules
        DESTINATION ${UDEV_RULES_PATH}
        COMPONENT "xtrx-drv"
        )
else (INSTALL_UDEV_RULES)
    message (STATUS "Udev rules not being installed, install them with -DINSTALL_UDEV_RULES=ON")
endif (INSTALL_UDEV_RULES)

include(CPack) #include last
