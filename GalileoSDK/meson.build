project('GalileoSDK', 'cpp',
    default_options : ['cpp_std=c++11', 'buildtype=release']
)
jsoncpp_dep = dependency('jsoncpp', required : true)
roscpp_dep = dependency('roscpp', required : true)
shared_library('GalileoSDK',
    'GalileoSDK.cpp',
    version : '1.0.0',
    soversion : '0',
    include_directories: include_directories('include', '../include'),
    dependencies: [jsoncpp_dep, roscpp_dep],
    install : true
)