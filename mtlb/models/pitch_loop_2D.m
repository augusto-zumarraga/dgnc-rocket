clear
clc

%%
a1 = sym('a1', 'real');
a2 = sym('a2', 'real');
a  = sym('a' , 'real');
b1 = sym('b1', 'real');

A = [a1 a2-b1*a ; 0 -a]

eig(A)

%%
a1 = sym('a1', 'real');
a2 = sym('a2', 'real');
a  = sym('a' , 'real');
b1 = sym('b1', 'real');

A = [a1-a*b1 a2 ; -a 0]

eig(A)

%%
a11 = sym('a11', 'real');
a12 = sym('a12', 'real');
a21 = sym('a21', 'real');
a22 = sym('a22', 'real');

A = [a11 a12 ; a21 a22]

eig(A)

%%
m  = sym('m' , 'real');
za = sym('za', 'real');
ma = sym('ma', 'real');
Iy = sym('Iy' , 'real');

M = [m-za 0 0 ; -ma Iy 0 ; 0 0 1]

inv(M)

