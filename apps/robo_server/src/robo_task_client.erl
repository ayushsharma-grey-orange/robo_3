-module(robo_task_client).

-export([submit/1]).

-define(SERVER, "localhost").
-define(PORT, 5555).

submit(Goal) ->
    {ok, Socket} = gen_tcp:connect(
        ?SERVER,
        ?PORT,
        [binary,
         {packet, 4},
         {active, false}]
    ),

    ok = gen_tcp:send(
        Socket,
        term_to_binary({task, Goal})
    ),

    {ok, Data} = gen_tcp:recv(Socket, 0),
    Response = binary_to_term(Data),

    gen_tcp:close(Socket),

    io:format(
        "Task ~p submitted: ~p~n",
        [Goal, Response]
    ),

    Response.
