$(document).ready(function() {

    const logs = [];
    let simulatorIsActive = false;
    let interfaceIsActive = false;
    pollServer();

    /**
     * Poll server every 5 seconds
     */
    function pollServer() {
        const accessToken = $('#access-token').val();

        if (accessToken) {
            $.ajax('poll', {
                data: JSON.stringify({access_token: accessToken}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    res.logs.forEach(log => {
                        if (!logs.includes(log)) {
                            logs.push(log);
                            $('.log-window').append(`<p>${log}</p>`);
                        }
                    });

                    if (simulatorIsActive !== res.simulator_state) {
                        simulatorIsActive = res.simulator_state;
                        simulatorIsActive 
                        ? $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator') 
                        : $('#launch-exit').removeClass('btn-danger').addClass('btn-primary').text('LAUNCH simulator');
                    }

                    if (interfaceIsActive !== res.interface_state) {
                        interfaceIsActive = res.interface_state;
                        interfaceIsActive 
                        ? $('#start-stop').removeClass('btn-primary').addClass('btn-danger').text('Send STOP signal') 
                        : $('#start-stop').removeClass('btn-danger').addClass('btn-primary').text('Send GO signal');
                    }
                },
            });
        }
        setTimeout(pollServer, 5000);
    }

    /**
     * Simulator launch exit button handler
     */
    $('#launch-exit').click(() => {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }
    
        const selectedTeam = $("input:radio[name ='team-select']:checked").val();
        if (selectedTeam === undefined) {
            alert('Select a team.');
            return
        }
    
        const selectedMission = $("input:radio[name ='mission-select']:checked").val();
        if (selectedMission === undefined) {
            alert('Select a mission.');
            return
        }

    
        simulatorIsActive 
        ? 
        $.ajax('simulator/exit', {
            data: JSON.stringify({access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                simulatorIsActive = false;
                logs.push(res.response);
                $('#launch-exit').removeClass('btn-danger').addClass('btn-primary').text('LAUNCH simulator') 
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        }) 
        :
        $.ajax('simulator/launch', {
            data: JSON.stringify({id: selectedTeam, mission: selectedMission, access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                simulatorIsActive = true;
                logs.push(res.response);
                $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator'); 
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        });
    });

    /**
     * ROS bridge start stop button handler
     */
    $('#start-stop').click(() => {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }

        interfaceIsActive
        ?
        $.ajax('mission/stop', {
            data: JSON.stringify({access_token: accessToken,  sender: 'operator'}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                interfaceIsActive = false;
                logs.push(res.response);
                $('#start-stop').removeClass('btn-danger').addClass('btn-primary').text('Send GO signal');
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        })     
        :    
        $.ajax('mission/start', {
            data: JSON.stringify({access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                interfaceIsActive = true;
                logs.push(res.response);
                $('#start-stop').removeClass('btn-primary').addClass('btn-danger').text('Send STOP signal'); 
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        });
    });
    
    /**
     * Reset button handler
     */
    $('#reset').click(function() {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }
    
        $.ajax('mission/reset', {
            data: JSON.stringify({access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: function(res) {
                logs.push(res.response);
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        });     
    });
});