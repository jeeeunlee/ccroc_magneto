function fbp = tpoly2beizierCoeff( ftp )

global t
ftp = collect(expand(ftp), t);

% f(t) = tp(1)*t^0 + tp(1)*t^1 + ... + tp(n)*t^n-1
%      = bp(1)*(1-t)^(n-1) + bp(2)*(1-t)^(n-2)*t^1 + ... + bp(n)*t^n-1  

n = length(ftp);
fbp=[];
dim=[];
for i=1:n    
    tp = coeffs(ftp(i),t,'All');
    dim = [dim; length(tp)];
end
maxdim = max(dim);

for i=1:n    
    tp = coeffs(ftp(i),t,'All');
    tp = tp(end:-1:1);
    tp = [tp, zeros(1,maxdim-dim(i))];
    bp = tpoly2bernstein( tp );
    
    bpi = bp;
    
    fbp = [fbp; bpi'];
end



end