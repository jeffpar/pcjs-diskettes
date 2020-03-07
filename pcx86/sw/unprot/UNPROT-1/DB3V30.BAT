Ren Dbase.exe Dbase
Debug <Db3v30.bat
Goto Exit

N Db3v30.Bin
L
Rcx
100
N Db3v30.Unp
w 300

N Dbase
L
N Db3v30.Unp
L 300
A 46d
Jmp 333

N Dbase
Rbx
1
Rcx
C950
w
q
:Exit
Ren Dbase Dbase.exe


