%%%-------------------------------------------------------------------
%% @doc robo_2 public API
%% @end
%%%-------------------------------------------------------------------

-module(robo_2_app).

-behaviour(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    robo_2_sup:start_link().

stop(_State) ->
    ok.

%% internal functions
