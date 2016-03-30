# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('v2x-lte', ['core', 'network', 'mobility', 'lte'])
    module.source = [
		'model/gn-address.cc',
		'model/gn-basic-transport-header.cc',
		'model/gn-common-header.cc',
		'model/location-table.cc',
		'model/v2x-mobility-model.cc',
		'model/v2x-client.cc',
    	'helper/v2x-client-helper.cc',
		]

    module_test = bld.create_ns3_module_test_library('v2x-lte')
    module_test.source = [
        'test/v2x-lte-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'v2x-lte'
    headers.source = [
    	'model/gn-address.h',
    	'model/gn-basic-transport-header.h',
    	'model/gn-common-header.h',
    	'model/location-table.h',
    	'model/v2x-mobility-model.h',
    	'model/v2x-client.h',
    	'helper/v2x-client-helper.h',
    	]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    # bld.ns3_python_bindings()
