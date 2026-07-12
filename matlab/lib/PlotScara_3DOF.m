function PlotScara_3DOF(Q,L,col,fig,ll)

q1 = Q(1); q2 = Q(2); q3 = Q(3);
l1 = L(1); l2 = L(2); l3 = L(3);

x1 = l1*cos(q1);
y1 = l1*sin(q1);

x2 = x1 + l2*cos(q1+q2);
y2 = y1 + l2*sin(q1+q2);

x3 = x2 + l3*cos(q1+q2+q3);
y3 = y2 + l3*sin(q1+q2+q3);

figure(fig); grid on;
set(ll,'XData', [0 x1 x2 x3], 'YData', [0 y1 y2 y3], 'color', col);
drawnow;

app = (l1 + l2 + l3)*1.1;
xlim([-app app]);
ylim([-app app]);
end