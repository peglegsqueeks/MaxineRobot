from enum import Enum


class FacialAnimation(Enum):
    """
    All the Facial Animations
    """

    advised = "advised"
    agegender = "agegender"
    anger = "anger"
    backwards = "backwards"
    better = "better"
    cautionmovingbackwards = "cautionmovingbackwards"
    cautionmovingforward = "cautionmovingforward"
    cautionroguerobots = "cautionroguerobots"
    chess = "chess"
    comewithme = "comewithme"
    compute = "compute"
    cross = "cross"
    danger = "danger"
    dangerwillrobinson = "dangerwillrobinson"
    directive = "directive"
    disagegender = "disagegender"
    disemotional = "disemotional"
    dizzy = "dizzy"
    emotional = "emotional"
    forwards = "forwards"
    found_person = "found-person"
    found_you = "found-you"
    gosomewhere2 = "gosomewhere2"
    greetings = "greetings"
    hairybaby = "hairybaby"
    happy = "happy"
    helpme = "helpme"
    humans = "humans"
    inmyway = "inmyway"
    left = "left"
    lowbattery = "lowbattery"
    malfunction2 = "malfunction2"
    movingback = "movingback"
    movingforward = "movingforward"
    movingleft = "movingleft"
    movingright = "movingright"
    neutral = "neutral"
    nicesoftware2 = "nicesoftware2"
    no5alive = "no5alive"
    objective = "objective"
    powerdown = "powerdown"
    program = "program"
    right = "right"
    robotnotoffended = "robotnotoffended"
    sad = "sad"
    satisfiedwithmycare = "satisfiedwithmycare"
    search_on = "search-on"
    search_person = "search-person"
    searchmode_off = "searchmode-off"
    searchterminated = "searchterminated"
    seeyou = "seeyou"
    selfdestruct = "selfdestruct"
    shallweplayagame = "shallweplayagame"
    silly = "silly"
    stare = "stare"
    surprise = "surprise"
    waitbeforeswim = "waitbeforeswim"
    world = "world"

    def __str__(self) -> str:
        """
        Returns a string version of the animation
        """
        return f"Animating: {self.name}"

    @staticmethod
    def from_name(name: str) -> "FacialAnimation":
        """
        Loads an animation from a string name
        """

        # loop over all animations looking for this name
        for animation in FacialAnimation:
            if animation.name == name:
                return animation
