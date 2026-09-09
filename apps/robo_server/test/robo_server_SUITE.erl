-module(robo_server_SUITE).

-include_lib("common_test/include/ct.hrl").
-include_lib("stdlib/include/assert.hrl").

-export([all/0, init_per_suite/1, end_per_suite/1,
         init_per_testcase/2, end_per_testcase/2]).

-export([
    t_register_robot/1,
    t_submit_task_assigns_idle_robot/1,
    t_full_path_walk_triggers_prefetch/1,
    t_queued_task_auto_assigned_when_robot_frees_up/1,
    t_move_denied_triggers_replan/1,
    t_robot_disconnect_requeues_task/1,
    t_unreachable_goal_never_sent_to_robot/1
]).

-define(TEST_PORT, 15599).

%%--------------------------------------------------------------------
%% Suite setup
%%--------------------------------------------------------------------

all() ->
    [
        t_register_robot,
        t_submit_task_assigns_idle_robot,
        t_full_path_walk_triggers_prefetch,
        t_queued_task_auto_assigned_when_robot_frees_up,
        t_move_denied_triggers_replan,
        t_robot_disconnect_requeues_task,
        t_unreachable_goal_never_sent_to_robot
    ].

init_per_suite(Config) ->
    %% A dedicated test port + small, predictable obstacle set so path
    %% assertions below are deterministic. Adjust the obstacle list if
    %% your grid bounds differ from a 9x9 plane.
    application:set_env(robo_server, tcp_port, ?TEST_PORT),
    application:set_env(robo_server, window_size, 3),
    application:set_env(robo_server, obstacles, [{9, 3}, {8, 3}, {7, 3}, {7, 2}]),
    Config.

end_per_suite(_Config) ->
    ok.

init_per_testcase(_TestCase, Config) ->
    {ok, Pid} = robo_server:start_link(),
    [{server_pid, Pid} | Config].

end_per_testcase(_TestCase, Config) ->
    Pid = ?config(server_pid, Config),
    case is_process_alive(Pid) of
        true -> catch gen_server:stop(robo_server);
        false -> ok
    end,
    %% give the OS a moment to release the listening socket before
    %% the next testcase binds the same port again
    timer:sleep(50),
    ok.

%%--------------------------------------------------------------------
%% Helpers
%%--------------------------------------------------------------------

connect() ->
    {ok, Socket} =
        gen_tcp:connect(
            "localhost", ?TEST_PORT,
            [binary, {packet, 4}, {active, false}]
        ),
    Socket.

send_term(Socket, Term) ->
    ok = gen_tcp:send(Socket, term_to_binary(Term)).

recv_term(Socket, TimeoutMs) ->
    case gen_tcp:recv(Socket, 0, TimeoutMs) of
        {ok, Data} -> binary_to_term(Data);
        {error, Reason} -> ct:fail({recv_failed, Reason})
    end.

%% Asserts nothing arrives within TimeoutMs -- used to prove the
%% server correctly withheld a message (e.g. an unreachable task).
assert_no_message(Socket, TimeoutMs) ->
    case gen_tcp:recv(Socket, 0, TimeoutMs) of
        {error, timeout} -> ok;
        {ok, Data} -> ct:fail({unexpected_message, binary_to_term(Data)})
    end.

register_robot(RobotId, Pos) ->
    Socket = connect(),
    send_term(Socket, {hello, RobotId, Pos}),
    ?assertEqual({hello_ack, RobotId}, recv_term(Socket, 2000)),
    Socket.

%%--------------------------------------------------------------------
%% Test cases
%%--------------------------------------------------------------------

t_register_robot(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),
    gen_tcp:close(Socket),
    ok.

t_submit_task_assigns_idle_robot(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),
    {ok, _TaskId} = robo_server:submit_task({4, 1}),
    ?assertEqual({go, {4, 1}, [{2, 1}, {3, 1}, {4, 1}]}, recv_term(Socket, 2000)),
    gen_tcp:close(Socket),
    ok.

%% window_size is 3 and the path is 4 steps long, so this exercises
%% the exact "second-to-last cell of the window" prefetch trigger
%% we built into maybe_prefetch_next_window/3.
t_full_path_walk_triggers_prefetch(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),
    {ok, _TaskId} = robo_server:submit_task({5, 1}),

    ?assertEqual({go, {5, 1}, [{2, 1}, {3, 1}, {4, 1}]}, recv_term(Socket, 2000)),

    %% Step 1 of 3: nothing extra should arrive yet.
    send_term(Socket, {moved, <<"r1">>, {2, 1}}),
    ?assertEqual({move_ack, {2, 1}}, recv_term(Socket, 2000)),
    assert_no_message(Socket, 200),

    %% Step 2 of 3: this leaves exactly ONE cell ({4,1}) unwalked, so
    %% the server should prefetch+send the next window RIGHT HERE --
    %% before we've even reported moving to {4,1}.
    send_term(Socket, {moved, <<"r1">>, {3, 1}}),
    ?assertEqual({move_ack, {3, 1}}, recv_term(Socket, 2000)),
    ?assertEqual({go, {5, 1}, [{5, 1}]}, recv_term(Socket, 2000)),

    %% Step 3 of 3 (the original window's last cell): just an ack,
    %% no further prefetch since {5,1} is the goal itself.
    send_term(Socket, {moved, <<"r1">>, {4, 1}}),
    ?assertEqual({move_ack, {4, 1}}, recv_term(Socket, 2000)),
    assert_no_message(Socket, 200),

    %% Final step: arrival at the goal.
    send_term(Socket, {moved, <<"r1">>, {5, 1}}),
    ?assertEqual({goal_reached, {5, 1}}, recv_term(Socket, 2000)),

    gen_tcp:close(Socket),
    ok.

t_queued_task_auto_assigned_when_robot_frees_up(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),

    {ok, _T1} = robo_server:submit_task({3, 1}),
    ?assertEqual({go, {3, 1}, [{2, 1}, {3, 1}]}, recv_term(Socket, 2000)),

    %% Robot is now busy -- this second task must queue, not assign.
    {ok, _T2} = robo_server:submit_task({3, 3}),
    assert_no_message(Socket, 200),

    %% Walk robot to its first goal.
    send_term(Socket, {moved, <<"r1">>, {2, 1}}),
    ?assertEqual({move_ack, {2, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {moved, <<"r1">>, {3, 1}}),
    ?assertEqual({goal_reached, {3, 1}}, recv_term(Socket, 2000)),

    %% The queued task should now be assigned automatically -- no
    %% second submit_task call.
    ?assertMatch({go, {3, 3}, _Window}, recv_term(Socket, 2000)),

    gen_tcp:close(Socket),
    ok.

t_move_denied_triggers_replan(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),
    {ok, _TaskId} = robo_server:submit_task({4, 1}),
    ?assertEqual({go, {4, 1}, [{2, 1}, {3, 1}, {4, 1}]}, recv_term(Socket, 2000)),

    %% Simulate another robot having grabbed {2,1} out from under us
    %% by writing directly into the (public, named) reservations table.
    true = ets:insert(robo_reservations, {{2, 1}, <<"intruder">>}),

    send_term(Socket, {moved, <<"r1">>, {2, 1}}),
    ?assertEqual(
        {move_denied, {position_reserved_by, <<"intruder">>}},
        recv_term(Socket, 2000)
    ),

    %% Server should recover with a fresh window that avoids {2,1}.
    {go, {4, 1}, NewWindow} = recv_term(Socket, 2000),
    ?assert(not lists:member({2, 1}, NewWindow)),

    gen_tcp:close(Socket),
    ok.

t_robot_disconnect_requeues_task(_Config) ->
    SocketA = register_robot(<<"robot-a">>, {1, 1}),
    {ok, _TaskId} = robo_server:submit_task({4, 1}),
    ?assertEqual({go, {4, 1}, [{2, 1}, {3, 1}, {4, 1}]}, recv_term(SocketA, 2000)),

    %% Robot A vanishes mid-task without finishing.
    gen_tcp:close(SocketA),
    timer:sleep(150),

    %% A fresh robot connecting should pick up the orphaned task
    %% automatically.
    SocketB = register_robot(<<"robot-b">>, {1, 1}),
    ?assertEqual({go, {4, 1}, [{2, 1}, {3, 1}, {4, 1}]}, recv_term(SocketB, 2000)),

    gen_tcp:close(SocketB),
    ok.

t_unreachable_goal_never_sent_to_robot(_Config) ->
    Socket = register_robot(<<"r1">>, {1, 1}),
    %% {9,3} is one of the configured static obstacles -- no path can
    %% possibly reach it.
    {ok, _TaskId} = robo_server:submit_task({9, 3}),
    assert_no_message(Socket, 500),
    gen_tcp:close(Socket),
    ok.