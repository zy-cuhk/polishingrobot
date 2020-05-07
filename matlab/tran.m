function X=tran(v,l1,l2,l3,l4);
s1=sin(v(1)); c1=cos(v(1));
s12=sin(v(1)+v(2)); c12=cos(v(1)+v(2));
s123=sin(v(1)+v(2)+v(3)); c123=cos(v(1)+v(2)+v(3));
s1234=sin(v(1)+v(2)+v(3)+v(4)); c1234=cos(v(1)+v(2)+v(3)+v(4));
x=l1*c1+l2*c12+l3*c123+l4*c1234;
y=l1*s1+l2*s12+l3*s123+l4*s1234;
X=[x y]';