mdp

module camera
    c: [0..1];
    r: [1..3];
    [goto0] true -> (c'=0) & (r'=1); 
    [goto1] true -> (c'=0) & (r'=2); 
    [goto2] true -> (c'=0) & (r'=3);
    [goto3] true -> (c'=1) & (r'=1);
    [goto4] true -> (c'=1) & (r'=2);
    [goto5] true -> (c'=1) & (r'=3);
endmodule

label "goal" = c=1 & r!=1;

rewards "energy"
    [goto0]c=1:1;
    [goto0]c=0:0.1;
    [goto1]c=1:2;
    [goto1]c=0:0.2;
    [goto2]c=1:1;
    [goto2]c=0:0.1;
    [goto3]c=0:3;
    [goto3]c=1:0.3;
    [goto4]c=0:1;
    [goto4]c=1:0.1;
    [goto5]c=0:4;
    [goto5]c=1:0.4;
endrewards
