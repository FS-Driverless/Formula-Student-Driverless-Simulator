$(document).ready(function() {

    // Start button handler
    $('#start').click(function() {
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
            data: JSON.stringify({id: selectedTeam, mission: selectedMission}),
            contentType: 'application/json',
            type: 'POST',
            success: function(res) {
                alert(res);
                $('.log-window').html(res);
            }
        });
    });

    // Stop button handler
    $('#stop').click(function() {
        $.ajax('mission/stop', {
            type: 'POST'
        });     
    });

    // Reset button handler
    $('#reset').click(function() {
        $.ajax('mission/reset', {
            type: 'POST'
        });     
    });
});