^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package async_comm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix for UDP/TCP when loopback is only device with an address
* Added listener interface
* Added custom message handler functionality
* TCPClient class implementing an async tcp client
* Contributors: Daniel Koch, James Jackson, Rein Appeldoorn

0.1.1 (2019-02-21)
------------------
* Some cleanup
* Process callbacks on a separate thread
* Made buffer sizes constant, removed faulty mechanism for overriding
* Removed requirement that bulk write length be no greater than write buffer size
* Contributors: Daniel Koch

0.1.0 (2018-08-31)
------------------
* Initial release
* Contributors: Daniel Koch, James Jackson
