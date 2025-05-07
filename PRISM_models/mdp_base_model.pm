mdp

module main
    c: [0..1];
    r: [1..2];

    total_pics: [0..10];

    [config0] total_pics < 10 -> (c'=0) & (r'=1) & (total_pics'=total_pics+1);
    [config1] total_pics < 9 -> (c'=0) & (r'=2) & (total_pics'=total_pics+2);
    [config2] total_pics < 10 -> (c'=1) & (r'=1) & (total_pics'=total_pics+1);
    [config3] total_pics < 9 -> (c'=1) & (r'=2) & (total_pics'=total_pics+2);
endmodule

label "goal" = total_pics = 10; 

rewards
    c=0 : r;
    c=1 : 2*r;
endrewards
