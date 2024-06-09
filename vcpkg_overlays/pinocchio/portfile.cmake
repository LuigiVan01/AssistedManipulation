# vcpkg_from_github(
#     OUT_SOURCE_PATH SOURCE_PATH
#     REPO stack-of-tasks/pinocchio
#     REF "v${VERSION}"
#     SHA512 4b41943c42d9971bf71afd4b9491c8faed2f69c844a69f934e2a951ccc3b012139df4cfafa76ae9454078c753bb406befc2e6738a264519152639dea70e305dc
#     HEAD_REF master
# )

find_program(GIT git)
set(GIT_URL "https://github.com/stack-of-tasks/pinocchio.git")
set(GIT_REV "v2.7.1")

# https://github.com/microsoft/vcpkg/issues/6886
set(SOURCE_PATH ${CURRENT_BUILDTREES_DIR}/src/${PORT})

if(NOT EXISTS "${SOURCE_PATH}/.git")
	message(STATUS "Cloning and fetching submodules")
	vcpkg_execute_required_process(
	  COMMAND ${GIT} clone --recurse-submodules ${GIT_URL} ${SOURCE_PATH}
	  WORKING_DIRECTORY ${CURRENT_BUILDTREES_DIR}
	  LOGNAME clone
	)

	message(STATUS "Checkout revision ${GIT_REV}")
	vcpkg_execute_required_process(
	  COMMAND ${GIT} checkout ${GIT_REV}
	  WORKING_DIRECTORY ${SOURCE_PATH}
	  LOGNAME checkout
	)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_WITH_URDF_SUPPORT=ON
        -DBUILD_UTILS=OFF
        -DBUILD_PYTHON_INTERFACE=OFF
        -DGENERATE_PYTHON_STUBS=OFF
        -DBUILD_WITH_COMMIT_VERSION=OFF
		-DBUILD_TESTING=OFF
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
