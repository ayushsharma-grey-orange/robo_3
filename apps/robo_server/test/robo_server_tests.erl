-module(robo_server_tests).
-include_lib("eunit/include/eunit.hrl").

next_positions_basic_test() ->
    Path = [{1,1},{2,1},{3,1},{4,1},{5,1}],
    ?assertEqual({ok, [{2,1},{3,1},{4,1}]}, robo_server:next_positions({1,1}, Path, 3)).

next_positions_near_goal_test() ->
    Path = [{1,1},{2,1}],
    ?assertEqual({ok, [{2,1}]}, robo_server:next_positions({1,1}, Path, 3)).

next_positions_at_goal_test() ->
    Path = [{1,1}],
    ?assertEqual({error, goal_reached}, robo_server:next_positions({1,1}, Path, 3)).

expected_next_test() ->
    ?assertEqual({ok, {2,1}}, robo_server:expected_next({1,1}, [{1,1},{2,1},{3,1}])).