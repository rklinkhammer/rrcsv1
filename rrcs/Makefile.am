include $(top_srcdir)/common.am
bin_PROGRAMS = rrcsv1
rrcsv1_SOURCES = rrcs_accelerometer.cpp rrcs_application.cpp rrcs_barometer.cpp rrcs_config.cpp rrcs.cpp rrcs_dashboard.cpp rrcs_kalman_filter.cpp rrcs_sensor.cpp rrcs_state.cpp fftsg.c 
rrcsv1_LDADD = $(top_builddir)/mraa/libmraa.a -lwt -lwthttp -lmraa
