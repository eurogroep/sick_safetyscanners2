^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safetyscanners2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'change/8-char-checksums' into lowpad
* change(diagnostics): checksums 8 char long
  In order to align the checksum string with the Sick safety designer.
* fix: version char
* Contributors: Rein Appeldoorn

2.0.1 (2023-09-19)
------------------
* fix: version char
* Contributors: Rein Appeldoorn

2.0.0 (2023-09-13)
------------------
* feat(diagnostics): diagnosed scan publisher
* refactor: combine Node and LifeCycle node implementations
  Also added ros2 clang format file to auto format code.
* Contributors: Rein Appeldoorn

1.0.3 (2021-12-22)
------------------
* Fixes unsafe pointer access in UDP callback
* Implement lifecycle node 
* Added functionality to allow multicast
* set not using the default sick angles as default
* moved changeSensor settings to be always be invoked
* fixed typo in launch file
* Contributors: Brice, Erwin Lejeune, Soma Gallai, Lennart Puck, Tanmay

1.0.2 (2021-03-15)
------------------
* added missing dependencies to package xml
* Contributors: Lennart Puck

1.0.1 (2021-03-05)
------------------
* changed the parameter callback interface so its only triggered
  when the parameters of this node are called
* Contributors: Lennart Puck

1.0.0 (2021-01-11)
------------------

* Initial Release
