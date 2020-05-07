function dX=jac(v,l1,l2,l3,l4);
s1=sin(v(1)); c1=cos(v(1));
s12=sin(v(1)+v(2)); c12=cos(v(1)+v(2));
s123=sin(v(1)+v(2)+v(3)); c123=cos(v(1)+v(2)+v(3));
s1234=sin(v(1)+v(2)+v(3)+v(4)); c1234=cos(v(1)+v(2)+v(3)+v(4));
Jacob(1,1)=-l1*s1-l2*s12-l3*s123-l4*s1234;
Jacob(2,1)=l1*c1+l2*c12+l3*c123+l4*c1234;
Jacob(1,2)=-l2*s12-l3*s123-l4*s1234;
Jacob(2,2)=l2*c12+l3*c123+l4*c1234;
Jacob(1,3)=-l3*s123-l4*s1234;
Jacob(2,3)=l3*c123+l4*c1234;
Jacob(1,4)=-l4*s1234;
Jacob(2,4)=l4*c1234;

dX=Jacob*[v(5) v(6) v(7) v(8)]';