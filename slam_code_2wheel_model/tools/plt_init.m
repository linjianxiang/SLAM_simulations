function plt_name = plt_init(shape2d,color,linestyle,marker)
     plt_name = line(...
        'linestyle',linestyle,...
        'marker',marker,...
        'color',color,...
        'xdata',shape2d(1,:),...
        'ydata',shape2d(2,:));
end