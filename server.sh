
printf "\033]0;%s\007" "server"
echo "Starting server..."

echo -e "\033]633;P;TaskName=server\a"
rebar3 shell --apps robo_server