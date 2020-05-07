function Jacob=Jh(v,l1,l2,l3,l4);
s1=sin(v(1)); c1=cos(v(1));
s12=sin(v(1)+v(2)); c12=cos(v(1)+v(2));
s123=sin(v(1)+v(2)+v(3)); c123=cos(v(1)+v(2)+v(3));
s1234=sin(v(1)+v(2)+v(3)+v(4)); c1234=cos(v(1)+v(2)+v(3)+v(4));
J11=-l1*s1-l2*s12-l3*s123-l4*s1234;
J21=l1*c1+l2*c12+l3*c123+l4*c1234;
J12=-l2*s12-l3*s123-l4*s1234;
J22=l2*c12+l3*c123+l4*c1234;
J13=-l3*s123-l4*s1234;
J23=l3*c123+l4*c1234;
J14=-l4*s1234;
J24=l4*c1234;
Jacob=[J11 J12 J13 J14
       J21 J22 J23 J24];