# -*- coding: utf-8 -*-
from math import sqrt
N=29 # This the number we want to decompose
x=[]

def prime(a):
    judge=0
    if(a==2):
        return True
    else:
        for j in range(2,round(sqrt(a))+1):
            if(a%j==0):
                judge=1
            break
        if(judge==0):
            return True
        else:
            return False

def decompose(N):
    for i in range(2,round(sqrt(N))+1,1):
        if(N%i==0):
            x.append(i)
            break
    N=N/i
    if(prime(N)==False):
        decompose(N)
    else:
        x.append(int(N))
    if(len(x)==1):
        return N*i
    else:
        return x


x1=decompose(N)
print(x1) 

