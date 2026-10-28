# Optional detector DLL. Not linked into mpv/libmpv; loaded only after explicit
# addon activation. The legacy analysis core is GPL-2.0-or-later.
add_languages('cpp', required: true)
mv_cpp = meson.get_compiler('cpp')
mv_cpp_args = ['-DAJN_MVTOOLS_BUILD', '-D_CRT_SECURE_NO_WARNINGS']
mv_cpp_args += mv_cpp.get_id() == 'msvc' ? ['/FIcstddef'] : ['-include', 'cstddef']
mv_link_args = mv_cpp.get_id() == 'gcc' ? ['-static-libstdc++', '-static-libgcc'] : []
mvtools_library = shared_library('ajn-mvtools',
    files('detector.cpp', 'vendor/CopyCode.cpp', 'vendor/Fakery.cpp',
          'vendor/GroupOfPlanes.cpp', 'vendor/Luma.cpp', 'vendor/MVFrame.cpp',
          'vendor/PlaneOfBlocks.cpp', 'vendor/SADFunctions.cpp'),
    name_prefix: '', cpp_args: mv_cpp_args, link_args: mv_link_args,
    install: true, install_dir: get_option('bindir'))
install_data('LICENSE', 'UPSTREAM.md', install_dir: join_paths(get_option('datadir'), 'licenses', 'ajn-mvtools'))
if get_option('tests')
    mvtools_test = executable('ajn-mvtools-test', 'test_detector.cpp', link_with: mvtools_library)
    test('ajn-mvtools-detector', mvtools_test)
endif
