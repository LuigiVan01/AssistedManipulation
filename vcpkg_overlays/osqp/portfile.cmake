find_program(GIT git)
set(GIT_URL "https://github.com/osqp/osqp")
set(GIT_REV "v1.0.0.beta1")

# https://github.com/microsoft/vcpkg/issues/6886
set(SOURCE_PATH ${CURRENT_BUILDTREES_DIR}/src/${PORT})

if(NOT EXISTS "${SOURCE_PATH}/.git")
	message(STATUS "Cloning and fetching submodules")
	vcpkg_execute_required_process(
	  COMMAND ${GIT} clone -b ${GIT_REV} --recursive ${GIT_URL} ${SOURCE_PATH}
	  WORKING_DIRECTORY ${CURRENT_BUILDTREES_DIR}
	  LOGNAME clone
	)

	# Make submodules safe and reinitialise them. The above command deletes
	# the cloned files.
	message(STATUS "Make submodules safe and resetting")
	vcpkg_execute_required_process(
	  COMMAND ${GIT} submodule foreach --recursive "git config --global --add safe.directory . && git reset --hard"
	  WORKING_DIRECTORY ${CURRENT_BUILDTREES_DIR}/src/osqp
	  LOGNAME safe
	)

	message(STATUS "Precloning qdldl")
	vcpkg_execute_required_process(
	  COMMAND ${GIT} clone -b v0.1.6 https://github.com/osqp/qdldl.git qdldl_sources
	  WORKING_DIRECTORY ${CURRENT_BUILDTREES_DIR}/src/osqp/algebra//_common/lin_sys/qdldl
	  LOGNAME safe
	)
endif()

vcpkg_cmake_configure(SOURCE_PATH "${SOURCE_PATH}")
vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
