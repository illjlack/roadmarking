
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qRoadMarking.h
		${CMAKE_CURRENT_LIST_DIR}/qSignExtractDlg.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
