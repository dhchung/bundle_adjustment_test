function read_point()
ptcld = load("../ba_cpp/points/point0.txt");
t = ptcld(:,1:3);
c = ptcld(:,4:6);

figure(1);
scatter3(t(:,1), t(:,2), t(:,3), 10, c/255, 'filled');
axis equal;

end