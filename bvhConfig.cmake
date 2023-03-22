# Locate BVH dir
SET(ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Look for the header file.
SET(INCLUDE_DIR "${ROOT_DIR}")

# Handle the QUIETLY and REQUIRED arguments and set BVH_FOUND to TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	bvh DEFAULT_MSG
	ROOT_DIR
	INCLUDE_DIR)

# Copy the results to the output variables.
IF (BVH_FOUND)
	SET(BVH_INCLUDE_DIRS ${INCLUDE_DIR})
ELSE (BVH_FOUND)
	SET(BVH_INCLUDE_DIRS)
ENDIF (BVH_FOUND)

MARK_AS_ADVANCED(BVH_INCLUDE_DIRS)