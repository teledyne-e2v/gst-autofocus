project('gst-autofocus', 'c',
  version : '2.3.0',
  meson_version : '>= 0.43',
  default_options : [ 'warning_level=1',
                      'buildtype=debugoptimized' ])

gst_version = '1.14.2'
version_arr = gst_version.split('.')
gst_version_major = version_arr[0].to_int()
gst_version_minor = version_arr[1].to_int()
gst_version_micro = version_arr[2].to_int()
 if version_arr.length() == 4
  gst_version_nano = version_arr[3].to_int()
else
  gst_version_nano = 0
endif
gst_version_is_dev = gst_version_minor % 2 == 1 and gst_version_micro < 90



glib_req = '>= 2.56.0'
orc_req = '>= 0.4.17'
gst_req = '>= @0@.@1@.0'.format(gst_version_major, gst_version_minor)

api_version = '1.0'

plugins_install_dir = '/usr/local/lib/gstreamer-1.0'
plugins = []

cc = meson.get_compiler('c')
host_system = host_machine.system()

if cc.get_id() == 'msvc'
  msvc_args = [
      # Ignore several spurious warnings for things gstreamer does very commonly
      # If a warning is completely useless and spammy, use '/wdXXXX' to suppress it
      # If a warning is harmless but hard to fix, use '/woXXXX' so it's shown once
      # NOTE: Only add warnings here if you are sure they're spurious
      '/wd4018', # implicit signed/unsigned conversion
      '/wd4146', # unary minus on unsigned (beware INT_MIN)
      '/wd4244', # lossy type conversion (e.g. double -> int)
      '/wd4305', # truncating type conversion (e.g. double -> float)
      cc.get_supported_arguments(['/utf-8']), # set the input encoding to utf-8

      # Enable some warnings on MSVC to match GCC/Clang behaviour
      '/w14062', # enumerator 'identifier' in switch of enum 'enumeration' is not handled
      '/w14101', # 'identifier' : unreferenced local variable
      '/w14189', # 'identifier' : local variable is initialized but not referenced
  ]
  if have_cxx
    add_project_arguments(msvc_args, language: ['c', 'cpp'])
  else
    add_project_arguments(msvc_args, language: 'c')
  endif
  # Disable SAFESEH with MSVC for plugins and libs that use external deps that
  # are built with MinGW
  noseh_link_args = ['/SAFESEH:NO']
else
  noseh_link_args = []
endif

# Symbol visibility
if cc.has_argument('-fvisibility=hidden')
  add_project_arguments('-fvisibility=hidden', language: 'c')
endif

# Disable strict aliasing
if cc.has_argument('-fno-strict-aliasing')
  add_project_arguments('-fno-strict-aliasing', language: 'c')
endif

# Define G_DISABLE_DEPRECATED for development versions
if gst_version_is_dev
  message('Disabling deprecated GLib API')
  add_project_arguments('-DG_DISABLE_DEPRECATED', language: 'c')
endif

cdata = configuration_data()

check_headers = [
  ['HAVE_STDBOOL_H', 'stdbool.h'],
  ['HAVE_STDLIB_H', 'stdlib.h'],
  ['HAVE_UNISTD_H', 'unistd.h'],
  ['HAVE_STDIO_H', 'stdio.h'],
  ['HAVE_MATH_H', 'math.h'],
  ['HAVE_GSL_VECTOR_H', 'gsl/gsl_vector.h'],
  ['HAVE_GSL_MATRIX_H', 'gsl/gsl_matrix.h'],
  ['HAVE_GSL_BLAS_H', 'gsl/gsl_blas.h'],
  ['HAVE_GSL_MULTIFIT_NLINEAR_H', 'gsl/gsl_multifit_nlinear.h'],
  ['HAVE_GSL_RNG_H', 'gsl/gsl_rng.h'],
  ['HAVE_GSL_RANDIST_H', 'gsl/gsl_randist.h'],
  ['HAVE_PTHREAD_H', 'pthread.h'],
  ['HAVE_STRING_H', 'string.h'],
  ['HAVE_TIME_H', 'sys/time.h'],
  ['HAVE_SIGNAL_H', 'signal.h'],
  ['HAVE_PTHREAD_H', 'pthread.h'],
  ['HAVE_GST_H', 'gst/gst.h'],
  ['HAVE_APRA_INET_H', 'arpa/inet.h'], #i2c control
  ['HAVE_IOCTL_H', 'sys/ioctl.h'], #i2c control
  ['HAVE_FCNTL_H', 'fcntl.h'], #i2c control
  ['HAVE_STAT_H', 'sys/stat.h'], #i2c control
  ['HAVE_TYPES_H', 'sys/types.h'],#i2c control
  ['HAVE_I2C_H', 'linux/i2c.h'],#i2c control
  ['HAVE_I2V-DEV_H', 'linux/i2c-dev.h'],#i2c control
]


foreach h : check_headers
  if cc.has_header(h.get(1))
    cdata.set(h.get(0), 1)
  endif
endforeach

check_functions = [
  ['HAVE_LOG', 'log', '#include<math.h>'],
  ['HAVE_POW', 'pow', '#include<math.h>'],
]


libm = cc.find_library('m', required : true)

foreach f : check_functions
  if cc.has_function(f.get(1), prefix : f.get(2), dependencies : libm)
    cdata.set(f.get(0), 1)
  endif
endforeach

cdata.set('HAVE_IOS', host_system == 'ios')

cdata.set('SIZEOF_CHAR', cc.sizeof('char'))
cdata.set('SIZEOF_INT', cc.sizeof('int'))
cdata.set('SIZEOF_LONG', cc.sizeof('long'))
cdata.set('SIZEOF_SHORT', cc.sizeof('short'))
cdata.set('SIZEOF_VOIDP', cc.sizeof('void*'))
cdata.set('SIZEOF_OFF_T', cc.sizeof('off_t'))

# Here be fixmes.
# FIXME: check if this is correct
cdata.set('HAVE_CPU_X86_64', host_machine.cpu() == 'amd64')
cdata.set('HAVE_GCC_ASM', cc.get_id() != 'msvc')
cdata.set_quoted('VERSION', gst_version)
cdata.set_quoted('PACKAGE_VERSION', gst_version)
cdata.set_quoted('GST_LICENSE', 'LGPL')
cdata.set_quoted('PACKAGE', 'gst-plugins-good')
cdata.set_quoted('GETTEXT_PACKAGE', 'gst-plugins-good-1.0')
cdata.set_quoted('LOCALEDIR', join_paths(get_option('prefix'), get_option('localedir')))

warning_flags = [
  '-Wmissing-declarations',
  '-Wredundant-decls',
  '-Wwrite-strings',
  '-Winit-self',
  '-Wmissing-include-dirs',
  '-Wno-multichar',
  '-Wvla',
  '-Wall',
  '-Wpointer-arith',
]

warning_c_flags = [
  '-Wmissing-prototypes',
  '-Wdeclaration-after-statement',
  '-Wold-style-definition',
]


add_project_arguments('-O2','-fopenmp-simd', language: 'c')

foreach extra_arg : warning_flags
  if cc.has_argument (extra_arg)
    add_project_arguments([extra_arg], language: 'c')
  endif
endforeach

foreach extra_arg : warning_c_flags
  if cc.has_argument (extra_arg)
    add_project_arguments([extra_arg], language: 'c')
  endif
endforeach

# GStreamer package name and origin url
gst_package_name = 'autofocus plugin'

cdata.set_quoted('GST_PACKAGE_NAME', gst_package_name)


# Mandatory GST deps
gst_dep = dependency('gstreamer-1.0', version : gst_req,
  fallback : ['gstreamer', 'gst_dep'])
gstbase_dep = dependency('gstreamer-base-1.0', version : gst_req,
  fallback : ['gstreamer', 'gst_base_dep'])
gstapp_dep = dependency('gstreamer-app-1.0', version : gst_req,
    fallback : ['gst-plugins-base', 'app_dep'])
gstvideo_dep = dependency('gstreamer-video-1.0', version : gst_req,
    fallback : ['gst-plugins-base', 'video_dep'])

gsl_dep = dependency('gsl')


gst_plugins_good_args = ['-DHAVE_CONFIG_H','-lgsl','-lgslcblas','-lm']

configinc = include_directories('src/')

autofocus_sources = [
  'src/autofocusControl.c',
  'src/gstautofocus.c',
  'src/i2c.c',
  'src/i2c_control.c',
  'src/logger.c',
  'src/curve_fit.c',
]
thread_dep = dependency('threads')



gstautofocus = library('gstautofocus',
  autofocus_sources,
  c_args : gst_plugins_good_args,
  include_directories : [configinc],
  dependencies : [gstbase_dep,thread_dep,gsl_dep],
  install : true,
  install_dir : plugins_install_dir,
)

conf_data = configuration_data()
conf_data.set('package_version', meson.project_version())
conf_data.set('package_name', meson.project_name())
conf_data.set('package', meson.project_name())
conf_data.set('package_string', meson.project_name() + meson.project_version())
conf_data.set('tar_name', meson.project_name())
configure_file(input : 'config.h.in',
               output : 'config.h',
               configuration : conf_data)



