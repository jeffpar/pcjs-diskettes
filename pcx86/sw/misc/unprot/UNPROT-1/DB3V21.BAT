Ren Dbase.exe Dbase
Debug <db3v21.bat   
Goto Exit          

N Db3v21.Bin         
l
rCx                
100                
N Db3v21.Unp
w 300              

N Dbase
L                           
R                           
N Db3v21.unp
L 300                       
A 46d                       
Jmp 333

N Dbase                     
Rbx                         
1
Rcx
b850
w                           
q
:Exit                       
Ren Dbase Dbase.exe         

