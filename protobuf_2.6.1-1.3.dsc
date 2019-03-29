-----BEGIN PGP SIGNED MESSAGE-----
Hash: SHA256

Format: 3.0 (quilt)
Source: protobuf
Binary: libprotobuf9v5, libprotobuf-lite9v5, libprotobuf-dev, libprotoc9v5, libprotoc-dev, protobuf-compiler, python-protobuf, libprotobuf-java
Architecture: any all
Version: 2.6.1-1.3
Maintainer: Robert Edmonds <edmonds@debian.org>
Uploaders: Iustin Pop <iustin@debian.org>
Homepage: https://code.google.com/p/protobuf/
Standards-Version: 3.9.5
Vcs-Browser: http://anonscm.debian.org/gitweb/?p=collab-maint/protobuf.git
Vcs-Git: git://anonscm.debian.org/collab-maint/protobuf.git
Build-Depends: dpkg-dev (>= 1.16.1~), debhelper (>= 9), dh-autoreconf, g++ (>= 4:4.7), zlib1g-dev, dh-python, python-all (>= 2.7), libpython-all-dev (>= 2.7), python-setuptools, python-google-apputils, xmlto, unzip
Build-Depends-Indep: ant, default-jdk, maven-repo-helper
Package-List:
 libprotobuf-dev deb libdevel optional arch=any
 libprotobuf-java deb java optional arch=all
 libprotobuf-lite9v5 deb libs optional arch=any
 libprotobuf9v5 deb libs optional arch=any
 libprotoc-dev deb libdevel optional arch=any
 libprotoc9v5 deb libs optional arch=any
 protobuf-compiler deb devel optional arch=any
 python-protobuf deb python optional arch=any
Checksums-Sha1:
 375765455ad49e45e4e10364f91aaf2831d3e905 2641426 protobuf_2.6.1.orig.tar.gz
 07c4082b3d9b40dd12eba72cbd66e7a094b6f472 13116 protobuf_2.6.1-1.3.debian.tar.xz
Checksums-Sha256:
 dbbd7bdd2381633995404de65a945ff1a7610b0da14593051b4738c90c6dd164 2641426 protobuf_2.6.1.orig.tar.gz
 ef3867b3c34f5411c40c4ed1c25810c433de2c20359df16daa71cc123c5c3717 13116 protobuf_2.6.1-1.3.debian.tar.xz
Files:
 f3916ce13b7fcb3072a1fa8cf02b2423 2641426 protobuf_2.6.1.orig.tar.gz
 82f4cdfc61119e967ede5e8afa6301c3 13116 protobuf_2.6.1-1.3.debian.tar.xz

-----BEGIN PGP SIGNATURE-----

iQIcBAEBCAAGBQJV3jglAAoJEE3o/ypjx8yQmEMP/0lGsGnl76A2XOL1tt8p/L7B
vWbs2vhSfHPOZbxkOufhvLFZlr8V7ugxTETxY6fT2XlXcCdEqIUmouQtsQALBYe/
1r+s7naEaF7vmd3yf9svwt1o5TBtA0KhpOIGqbmxbOvTYDo/vqTZXyt4lDKByU2G
/CHwaRfo2j49Ql816h51hqgaP8HbrzkrGFqjC+IewpTYM8W2wUhlOyq8AchTa0B3
qanyUhZA1NqW8s5GyizPvkQKA/rgmTrsgd7HVKDTVVCvcPue9i94coE5qZkBGUBe
/9A0bCbDlYfBtR7fvjyYeeiGH47csxIrlLuvjGoBhUtnkSpc8QpLSfXc3u610Px1
jjdqJHkqATWPFvHPVhK6yyVjfzXpSaj5zRNkz1Td0dFpS+zeV41WCIOm7qxXw9mz
1cHd8jBtIUL7Yb+5zcp6R+TXcMs/YZHc11Q/WAn4L7CX3H1nC2R26FlfxSeD698n
s5e0JerUlu8iscU3mKriqbF8xdXypqTW8uu9hI0InXo96xVlbZJ6ZjxWpjlvgQlB
r7ztTltWwjebkNTIZsCW17ng7ko/LdybQW2PKr4c23rxJg1wy44wWY8XsMhyPYIr
Epu/WmkUK57udFn/Y8XalEefy42lzlckQo4HiVKg//T8g4lgEYfKKSIVWFBKLb5c
q7n2qgDgpqDrI6duRVYl
=ksjR
-----END PGP SIGNATURE-----
