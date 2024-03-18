# 🙋 FAQ

(faq-general)=
## General

### _/usr/bin/env: ‘python3\r’: No such file or directory_

$ rosrun lab1 mouse_client.py
/usr/bin/env: ‘python3\r’: No such file or directory


The problem are your line ending characters. Your file was created or edited on a Windows system and uses Windows/DOS-style line endings (CR+LF), whereas Linux systems like Ubuntu require Unix-style line endings (LF).

- Sublime: Open the desired file with Sublime and from the top menu select View -> Line Endings and then the Windows(CRLF) or Unix(LF). That’s it.
- VS Code: At the bottom right of the screen in VS Code there is a little button that says `LF` or `CRLF`: Click that button and change it to your preference.

<br>












