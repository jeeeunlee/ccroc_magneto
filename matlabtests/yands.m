function [ppy, pps] = yands(fw)

global y1 y2 y3 y

ppy = {};
pps = {};
for n=1:size(fw,2)
    pw = fw(:,n);
    py = [];
    ps = [];
	
    for i=1:6
       c1 = coeffs(pw(i),y1);
       c2 = coeffs(pw(i),y2);
       c3 = coeffs(pw(i),y3);
       cs = coeffs(pw(i),y);
      
       if(isempty(c1) || size(c1,2)==1)
           c1 = 0;
       else
           c1 = c1(end);
       end 
       if(isempty(c2)|| size(c2,2)==1)
           c2 = 0;
       else
           c2 = c2(end);
       end
       if(isempty(c3)|| size(c3,2)==1)
           c3 = 0;
       else
           c3 = c3(end);
       end
       if(isempty(cs))
           cs = 0;
       else
           cs = cs(1);
       end
       py = [py; [c1,c2,c3]];
       ps = [ps; cs];
    end
    ppy{n} = py;
    pps{n} = ps;
    
end