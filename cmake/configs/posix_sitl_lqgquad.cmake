include(cmake/configs/posix_sitl_simple.cmake)

list(APPEND config_module_list
	modules/lqg_quad
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS_lqgquad
	)
