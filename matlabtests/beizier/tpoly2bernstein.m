function bp = tpoly2bernstein( tp )

% f(t) = tp(1)*t^0 + tp(1)*t^1 + ... + tp(n)*t^n-1
%      = bp(1)*(1-t)^(n-1) + bp(2)*(1-t)^(n-2)*t^1 + ... + bp(n)*t^n-1  
n = length(tp);
syms bp [n 1] real;

for k=1:n
    bp(k) = tp(k);
    for i=1:(k-1)
        bp(k) = bp(k) - bp(i)*(-1)^(k-i)*nchoosek(n-i,k-i);
    end
end

for k=1:n
    bp(k)=bp(k)/nchoosek(n-1,k-1);
end


end