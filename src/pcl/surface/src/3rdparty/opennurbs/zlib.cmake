set(ZLIB_INCLUDES
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/crc32.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/deflate.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/inffast.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/inffixed.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/inflate.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/inftrees.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/trees.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/zconf.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/zlib.h
    include/pcl/${SUBSYS_NAME}/3rdparty/opennurbs/zutil.h
)

set(ZLIB_SOURCES
    src/3rdparty/opennurbs/adler32.c
    src/3rdparty/opennurbs/compress.c
    src/3rdparty/opennurbs/crc32.c
    src/3rdparty/opennurbs/deflate.c
    src/3rdparty/opennurbs/infback.c
    src/3rdparty/opennurbs/inffast.c
    src/3rdparty/opennurbs/inflate.c
    src/3rdparty/opennurbs/inftrees.c
    src/3rdparty/opennurbs/trees.c
    src/3rdparty/opennurbs/uncompr.c
    src/3rdparty/opennurbs/zutil.c
)
