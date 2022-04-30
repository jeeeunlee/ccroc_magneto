function [f1, f2] = DeCasteljau(fw, t0)

[d, n] = size(fw);
syms f1 [d n] real;
syms f2 [d n] real;
syms ftemp [n n] real;

for k=1:d
    for i=1:n
        ftemp(i,1) = fw(k,i);
    end
    for j=2:n
        for i=1:(n-j+1)
            ftemp(i,j) = ftemp(i,j-1)*(1-t0) + ftemp(i+1,j-1)*t0;
            if(i>n-j)
                break
            end
        end
    end
    
    
    for j=1:n
        f1(k,j) = ftemp(1, j);
        f2(k,j) = ftemp(j, n-j+1);
    end
end


end


%% check
bt0 = 0;
for j=1:n
    bt0 = bt0 + fw(k,j)*nchoosek(n-1,j-1)*(1-t0)^(n-j)*t0^(j-1);
end