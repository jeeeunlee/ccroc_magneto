clc; clear all;
addpath("beizier");

global t
global y1 y2 y3 y

syms g real %g=-9.81
syms t real

syms y [3 1] real
syms cs [3 1] real
syms cg [3 1] real

% y = [y1; y2; y3];
% cs =  [cs1; cs2; cs3];
% cg = [cg1; cg2; cg3];


ps = [cs';cs';cs';y';cg';cg';cg'];
% ps = [cs';cs';y';cg';cg'];
% ps = [cs';y';cg'];
n = length(ps);
c=0;
for i=1:n
    c = c + bernsteinbasis(n-1,i-1)*t^(i-1)*(1-t)^(n-i)*ps(i,:)';
end

ddc=0;
for i=1:n-2
    ddc = ddc + bernsteinbasis(n-3,i-1)*t^(i-1)*(1-t)^(n-2-i)*(ps(i+2,:)-2*ps(i+1,:)+ps(i,:))';
end

syms T2 real;

w = [ddc - [0;0;g]; cross(c,ddc-[0;0;g]) ];


fw = tpoly2beizierCoeff( w );
[ppy, pps] = yands(fw);

% t0 = 2/5; t1 = 2/3; 
% [f1, f2] = DeCasteljau(fw, t0);
% [f2, f3] = DeCasteljau(f2, t1);
% 
% [ppy1, pps1] = yands(f1);
% [ppy2, pps2] = yands(f2);
% [ppy3, pps3] = yands(f3);




