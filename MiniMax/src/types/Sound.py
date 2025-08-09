import pygame


from enum import Enum


class Sound(Enum):
    """
    The different sounds the robot can say
    """

    greetings = "DefaultSayings/greetings.wav"
    powerdown = "DefaultSayings/powerdown.wav"
    advised = "DefaultSayings/advised.wav"
    agegender = "DefaultSayings/agegender.wav"
    disagegender = "DefaultSayings/disagegender.wav"
    disemotional = "DefaultSayings/disemotional.wav"
    emotional = "DefaultSayings/emotional.wav"
    found_person = "DefaultSayings/found-person.wav"
    objective = "DefaultSayings/objective.wav"
    searchmode = "DefaultSayings/searchmode-off.wav"
    searchterminated = "DefaultSayings/searchterminated.wav"
    better = "DefaultSayings/better.wav"
    cautionroguerobots = "DefaultSayings/cautionroguerobots.wav"
    chess = "DefaultSayings/chess.wav"
    compute = "DefaultSayings/compute.wav"
    cross = "DefaultSayings/cross.wav"
    danger = "DefaultSayings/danger.wav"
    dangerwillrobinson = "DefaultSayings/dangerwillrobinson.wav"
    directive = "DefaultSayings/directive.wav"
    humans = "DefaultSayings/humans.wav"
    malfunction2 = "DefaultSayings/malfunction2.wav"
    nicesoftware2 = "DefaultSayings/nicesoftware2.wav"
    no5alive = "DefaultSayings/no5alive.wav"
    program = "DefaultSayings/program.wav"
    selfdestruct = "DefaultSayings/selfdestruct.wav"
    shallweplayagame = "DefaultSayings/shallweplayagame.wav"
    comewithme = "DefaultSayings/comewithme.wav"
    gosomewhere2 = "DefaultSayings/gosomewhere2.wav"
    hairybaby = "DefaultSayings/hairybaby.wav"
    lowbattery = "DefaultSayings/lowbattery.wav"
    robotnotoffended = "DefaultSayings/robotnotoffended.wav"
    satisfiedwithmycare = "DefaultSayings/satisfiedwithmycare.wav"
    waitbeforeswim = "DefaultSayings/waitbeforeswim.wav"
    silly = "DefaultSayings/silly.wav"
    stare = "DefaultSayings/stare.wav"
    world = "DefaultSayings/world.wav"
    anger = "DefaultSayings/anger.wav"
    backwards = "DefaultSayings/backwards.wav"
    cautionmovingbackwards = "DefaultSayings/cautionmovingbackwards.wav"
    cautionmovingforward = "DefaultSayings/cautionmovingforward.wav"
    dizzy = "DefaultSayings/dizzy.wav"
    forwards = "DefaultSayings/forwards.wav"
    found_you = "DefaultSayings/found-you.wav"
    happy = "DefaultSayings/happy.wav"
    helpme = "DefaultSayings/helpme.wav"
    inmyway = "DefaultSayings/inmyway.wav"
    left = "DefaultSayings/left.wav"
    movingback = "DefaultSayings/movingback.wav"
    movingforward = "DefaultSayings/movingforward.wav"
    movingleft = "DefaultSayings/movingleft.wav"
    movingright = "DefaultSayings/movingright.wav"
    neutral = "DefaultSayings/neutral.wav"
    right = "DefaultSayings/right.wav"
    sad = "DefaultSayings/sad.wav"
    search_on = "DefaultSayings/search-on.wav"
    search_person = "DefaultSayings/search-person.wav"
    seeyou = "DefaultSayings/seeyou.wav"
    surprise = "DefaultSayings/surprise.wav"
    two = "Sound/2.mp3"
    Powerup = "Sound/Powerup/Powerup_chirp2.mp3"
    questioning_computer = "Sound/Randombeeps/Questioning_computer_chirp.mp3"
    double_beep = "Sound/Randombeeps/Double_beep2.mp3"
    Powerdown = "Sound/Powerdown/Long_power_down.mp3"
    radar_bleep = "Sound/Radarscanning/Radar_bleep_chirp.mp3"
    radar_scanning = "Sound/Radarscanning/Radar_scanning_chirp.mp3"
    celebrate1 = "Sound/celebrate1.mp3"
    da_de_la = "Sound/Randombeeps/Da_de_la.mp3"

    def load_sound(self) -> pygame.mixer.Sound:
        """
        Loads a sound from file
        """
        return pygame.mixer.Sound(f"{self.value}")

    def __str__(self) -> str:
        return f"Play Sound: {self.name}"

    @staticmethod
    def from_name(name: str) -> "Sound":
        """
        Reuturns a sound from a name
        """
        # loop through all sounds looking for matching name
        for sound in Sound:
            if sound.name == name:
                return sound
