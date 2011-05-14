
create table pms_guess (
    id integer primary key autoincrement,
    ransac_iterations_count integer not null,
    min_inliers_count integer not null,
    max_projection_error float not null,
    -- constraints
    check (ransac_iterations_count >= 0),
    check (min_inliers_count >= 0),
    check (max_projection_error >= 0)
);

