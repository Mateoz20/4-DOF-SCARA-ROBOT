function p = fk_robot_13(q)

    L1  = 111.38;
    L22 = 19.75;
    L2  = 133.8;
    L33 = 1.25;
    L3  = 240;
    L44 = 20.13;
    L4  = 252.5;
    L55 = 10.95;
    L5  = 119.53;

    DH = [
        0       0         L1        q(1);
        0       0         L2        0;
        0       pi/2      0         0;
        0       0        -L22       q(2);
        0       0         0         pi/2;
        L3      0         0         0;
        0       0        -L33       q(3);
        0       0        -L44       0;
        0       0         0         pi/2;
        L4      0         0         q(4);
        L5      0         0         0;
        0      -pi/2      0         0;
        0       0        -L55       0
    ];

    T = eye(4);
    for i = 1:size(DH,1)
        a     = DH(i,1);
        alpha = DH(i,2);
        d     = DH(i,3);
        theta = DH(i,4);

        A = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  a*cos(theta);
             sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)  a*sin(theta);
             0           sin(alpha)             cos(alpha)             d;
             0           0                     0                      1];

        T = T * A;
    end

    p = T(1:3,4);
end
