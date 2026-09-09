-module(robo_pathfinder_SUITE).

-include_lib("common_test/include/ct.hrl").
-include_lib("stdlib/include/assert.hrl").

-export([all/0]).
-export([
    t_straight_line_no_obstacles/1,
    t_path_avoids_obstacles/1,
    t_unreachable_goal_reports_no_path/1,
    t_start_equals_goal/1,
    t_path_goal_blocked/1
]).

all() ->
    [
        t_straight_line_no_obstacles,
        t_path_avoids_obstacles,
        t_unreachable_goal_reports_no_path,
        t_start_equals_goal,
        t_path_goal_blocked
    ].

t_straight_line_no_obstacles(_Config) ->
    {ok, Path} = robo_pathfinder:find_path({1, 1}, {4, 1}, []),
    ?assertEqual({1, 1}, hd(Path)),
    ?assertEqual({4, 1}, lists:last(Path)),
    ok.

t_path_avoids_obstacles(_Config) ->
    Obstacles = [{2, 1}],
    {ok, Path} = robo_pathfinder:find_path({1, 1}, {3, 1}, Obstacles),
    ?assert(not lists:any(fun(P) -> lists:member(P, Obstacles) end, Path)),
    ok.

t_path_goal_blocked(_)->
    Obstacles = [{2, 1}, {3, 1}],
    ?assertEqual({error, no_path}, robo_pathfinder:find_path({1, 1}, {3, 1}, Obstacles)),
    ok.

t_unreachable_goal_reports_no_path(_Config) ->
    Wall = [{X, 5} || X <- lists:seq(1, 9)],
    ?assertEqual({error, no_path}, robo_pathfinder:find_path({1, 1}, {9, 9}, Wall)),
    ok.

t_start_equals_goal(_Config) ->
    {ok, Path} = robo_pathfinder:find_path({3, 3}, {3, 3}, []),
    ?assertEqual([{3, 3}], Path),
    ok.