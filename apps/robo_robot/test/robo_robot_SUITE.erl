-module(robo_robot_SUITE).

-include_lib("common_test/include/ct.hrl").
-include_lib("stdlib/include/assert.hrl").

-export([all/0, init_per_testcase/2, end_per_testcase/2]).

-export([
    t_registration_handshake/1,
    t_walk_window_sequentially/1,
    t_prefetched_go_is_deferred_then_used/1,
    t_move_denied_then_recovers_on_next_go/1
]).

all() ->
    [
        t_registration_handshake,
        t_walk_window_sequentially,
        t_prefetched_go_is_deferred_then_used,
        t_move_denied_then_recovers_on_next_go
    ].

init_per_testcase(_TestCase, Config) ->
    {ok, LSock} =
        gen_tcp:listen(0, [binary, {packet, 4}, {active, false}, {reuseaddr, true}]),
    {ok, Port} = inet:port(LSock),

    application:set_env(robo_robot, server_host, "localhost"),
    application:set_env(robo_robot, server_port, Port),
    application:set_env(robo_robot, step_delay_ms, 10),

    [{listen_socket, LSock} | Config].

end_per_testcase(_TestCase, Config) ->
    LSock = ?config(listen_socket, Config),
    catch gen_tcp:close(LSock),
    ok.

%%--------------------------------------------------------------------
%% Helpers
%%--------------------------------------------------------------------

send_term(Socket, Term) ->
    ok = gen_tcp:send(Socket, term_to_binary(Term)).

recv_term(Socket, TimeoutMs) ->
    case gen_tcp:recv(Socket, 0, TimeoutMs) of
        {ok, Data} -> binary_to_term(Data);
        {error, Reason} -> ct:fail({recv_failed, Reason})
    end.

%% Spawns robo_robot:start/2 (which loops forever) and hands back the
%% accepted socket -- the test acts as the "server" from here on.
start_robot_and_accept(Config, RobotId, StartPos) ->
    LSock = ?config(listen_socket, Config),
    RobotPid = spawn(robo_robot, start, [RobotId, StartPos]),
    {ok, FakeServerSocket} = gen_tcp:accept(LSock, 2000),
    {RobotPid, FakeServerSocket}.

do_handshake(Config, RobotId, StartPos) ->
    {RobotPid, Socket} = start_robot_and_accept(Config, RobotId, StartPos),
    ?assertEqual({hello, RobotId, StartPos}, recv_term(Socket, 2000)),
    send_term(Socket, {hello_ack, RobotId}),
    {RobotPid, Socket}.

%%--------------------------------------------------------------------
%% Test cases
%%--------------------------------------------------------------------

t_registration_handshake(Config) ->
    {RobotPid, Socket} = do_handshake(Config, <<"r1">>, {1, 1}),

    %% Confirm the robot is now idle-waiting by exercising the
    %% {wait, Reason} branch.
    send_term(Socket, {wait, testing}),
    ?assertEqual({wait_ack, <<"r1">>}, recv_term(Socket, 2000)),

    exit(RobotPid, kill),
    ok.

t_walk_window_sequentially(Config) ->
    {RobotPid, Socket} = do_handshake(Config, <<"r1">>, {1, 1}),

    send_term(Socket, {go, {3, 1}, [{2, 1}, {3, 1}]}),

    ?assertEqual({moved, <<"r1">>, {2, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {move_ack, {2, 1}}),

    ?assertEqual({moved, <<"r1">>, {3, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {goal_reached, {3, 1}}),

    exit(RobotPid, kill),
    ok.

%% This is the key test: it proves robo_robot correctly stashes an
%% unsolicited {go, ...} that arrives BEFORE it has finished its
%% current window, and acts on it immediately once the window is
%% done -- with no further message needed from the server.
t_prefetched_go_is_deferred_then_used(Config) ->
    {RobotPid, Socket} = do_handshake(Config, <<"r1">>, {1, 1}),

    send_term(Socket, {go, {5, 1}, [{2, 1}, {3, 1}, {4, 1}]}),

    ?assertEqual({moved, <<"r1">>, {2, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {move_ack, {2, 1}}),

    ?assertEqual({moved, <<"r1">>, {3, 1}}, recv_term(Socket, 2000)),
    %% Simulate the server's prefetch: ack for THIS move, immediately
    %% followed by the next window's `go' -- arriving before the
    %% robot has even attempted {4,1}.
    send_term(Socket, {move_ack, {3, 1}}),
    send_term(Socket, {go, {5, 1}, [{5, 1}]}),

    %% The robot must still finish the ORIGINAL window's last cell
    %% next, without waiting for any further server message.
    ?assertEqual({moved, <<"r1">>, {4, 1}}, recv_term(Socket, 500)),
    send_term(Socket, {move_ack, {4, 1}}),

    %% Now it should proceed straight into the pending window we
    %% queued earlier -- again, with no new `go' sent by us here.
    ?assertEqual({moved, <<"r1">>, {5, 1}}, recv_term(Socket, 500)),
    send_term(Socket, {goal_reached, {5, 1}}),

    exit(RobotPid, kill),
    ok.

t_move_denied_then_recovers_on_next_go(Config) ->
    {RobotPid, Socket} = do_handshake(Config, <<"r1">>, {1, 1}),

    send_term(Socket, {go, {2, 1}, [{2, 1}]}),
    ?assertEqual({moved, <<"r1">>, {2, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {move_denied, position_reserved_by_someone_else}),

    %% Robot must stay alive and keep listening for the server's
    %% follow-up plan instead of crashing or hanging up.
    send_term(Socket, {go, {2, 1}, [{2, 1}]}),
    ?assertEqual({moved, <<"r1">>, {2, 1}}, recv_term(Socket, 2000)),
    send_term(Socket, {goal_reached, {2, 1}}),

    exit(RobotPid, kill),
    ok.