function read_data()
	Pose = load("../ba_cpp/pose.txt");
    
    t = [];
    T = eye(4);
    t = [t T(1:3,4)];
    for i=1:size(Pose,1)
        Rt = Pose(i,:);
        Rt = reshape(Rt, [3, 4]);
        Tnew = [Rt; 0 0 0 1];
        T = Tnew * T;
        t = [t T(1:3,4)];
    end

    path = [t(1,:); t(3,:); -t(2,:)];

    path2 = t;
    
    figure(1);
    plot3(path(1,:), path(2,:), path(3,:));
    hold on;
    scatter3(path(1,1), path(2,1), path(3,1), 10, 'r', 'filled');
    scatter3(path(1,end), path(2,end), path(3,end), 10, 'b', 'filled');
    hold off;
    
    axis equal;
    xlabel X;
    ylabel Y;
    zlabel Z;

    T = eye(4);
    PT = [];
    C = [];
    for i=1:size(Pose,1)
        Rt = Pose(i,:);
        Rt = reshape(Rt, [3, 4]);
        Tnew = [Rt; 0 0 0 1];
        T = Tnew * T;
        ptcld = load("../ba_cpp/points/point"+num2str(i-1)+".txt");
        t = ptcld(:,1:3);
        c = ptcld(:,4:6);
        
        t2 = transform_pt(t, T);
        PT = [PT; t2];
        C = [C; c];

        figure(2);
        plot3(path2(1,:), path2(2,:), path2(3,:), 'b');
        hold on;
        scatter3(path2(1,i), path2(2,i), path2(3,i), 10, 'k', 'filled');
%         scatter3(t(:,1), t(:,2), t(:,3), 10, c/255, 'filled');
        scatter3(PT(:,1), PT(:,2), PT(:,3), 10, C/255, 'filled');
        hold off;
        
        

        axis equal;
        xlabel X;
        ylabel Y;
        zlabel Z;
        
        figure(3);
        C2 = [C(:,3), C(:,2), C(:,1)];
        scatter3(PT(:,1), PT(:,2), PT(:,3), 10, C2/255, 'filled');
        axis equal;
        xlabel X;
        ylabel Y;
        zlabel Z;
        view(0,-90);
        drawnow;
    end
    
end

function pt2 = transform_pt(pt, T)
temp = pt';
temp = [temp;ones(1, size(temp,2))];

temp = T*temp;
temp = temp(1:3,:)./temp(4,:);
temp = temp(1:3,:);
temp = temp';
pt2 = temp;
end