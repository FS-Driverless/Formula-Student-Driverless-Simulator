$(document).ready(function() {

    const logs = [];
    pollServer();
    
    // Poll logs from server every 5 seconds
    function pollServer() {
        $.ajax('logs', {
            type: 'GET',
            success: res => {
                res.response.forEach(log => {
                    if (!logs.includes(log)) {
                        logs.push(log);
                        $('.log-window').append(`<p>${log}</p>`);
                    }
                });
            }
        });
        setTimeout(pollServer, 5000);
    }
    
    // Start button handler
    $('#start').click(() => {
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
    
        $.ajax('mission/start', {
            data: JSON.stringify({id: selectedTeam, mission: selectedMission, access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                logs.push(res.response);
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        });
    });
    
    // Stop button handler
    $('#stop').click(() => {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }
    
        $.ajax('mission/stop', {
            data: JSON.stringify({access_token: accessToken}),
            contentType: 'application/json',
            type: 'POST',
            success: res => {
                logs.push(res.response);
                $('.log-window').append(`<p>${res.response}</p>`);
            },
            error: res => {
                alert(res.responseJSON.error);
            }
        });     
    });
    
    // Reset button handler
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