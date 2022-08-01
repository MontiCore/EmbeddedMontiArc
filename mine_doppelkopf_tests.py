# Used to mine some test games for the GDL model of Doppelkopf

import requests
from bs4 import BeautifulSoup
import re

vb_regex = re.compile(r"([\w ]+) \[")
ansage_regex = re.compile(r"([\w\d]+) \((\d)\)")

def mine_all_games():
    dk_url = 'https://www.online-doppelkopf.com/spiele'
    html_text = requests.get(dk_url).text
    soup = BeautifulSoup(html_text, 'html.parser')

    game_links = []

    for a in soup.find_all("a"):
        link = a.get("href")
        if link.startswith("/spiele/"):
            game_links.append(link[len("/spiele/"):])

    return game_links

imageLinkToCardDict = {
    "/images/kartenmax/clubq.jpg" : "kreuz_dame",
    "/images/kartenmax/clubj.jpg" : "kreuz_bube",
    "/images/kartenmax/cluba.jpg" : "kreuz_ass",
    "/images/kartenmax/club10.jpg" : "kreuz_zehn",
    "/images/kartenmax/clubk.jpg" : "kreuz_koenig",
    "/images/kartenmax/club9.jpg" : "kreuz_neun",

    "/images/kartenmax/spadeq.jpg" : "pik_dame",
    "/images/kartenmax/spadej.jpg" : "pik_bube",
    "/images/kartenmax/spadea.jpg" : "pik_ass",
    "/images/kartenmax/spade10.jpg" : "pik_zehn",
    "/images/kartenmax/spadek.jpg" : "pik_koenig",
    "/images/kartenmax/spade9.jpg" : "pik_neun",

    "/images/kartenmax/heartq.jpg" : "herz_dame",
    "/images/kartenmax/heartj.jpg" : "herz_bube",
    "/images/kartenmax/hearta.jpg" : "herz_ass",
    "/images/kartenmax/heart10.jpg" : "herz_zehn",
    "/images/kartenmax/heartk.jpg" : "herz_koenig",
    "/images/kartenmax/heart9.jpg" : "herz_neun",

    "/images/kartenmax/diamq.jpg" : "karo_dame",
    "/images/kartenmax/diamj.jpg" : "karo_bube",
    "/images/kartenmax/diama.jpg" : "karo_ass",
    "/images/kartenmax/diam10.jpg" : "karo_zehn",
    "/images/kartenmax/diamk.jpg" : "karo_koenig",
    "/images/kartenmax/diam9.jpg" : "karo_neun",
}
ansagen_map = {
    "Ko": "ansagen kontra",
    "K90": "absagen 90",
    "K60": "absagen 60",
    "K30": "absagen 30",
    "K0": "absagen schwarz",

    "Re": "ansagen re",
    "R90": "absagen 90",
    "R60": "absagen 60",
    "R30": "absagen 30",
    "R0": "absagen schwarz",
}
vorbehalt_map = {
    "Damensolo": "solo_damen",
    "Bubensolo": "solo_buben",
    "Farbsolo Kreuz": "solo_trumpf_kreuz",
    "Farbsolo Pik": "solo_trumpf_pik",
    "Farbsolo Herz": "solo_trumpf_herz",
    "Farbsolo Karo": "solo_trumpf_karo",
    "Assesolo": "solo_trumpf_ass",
    "Hochzeit": "hochzeit",
}

def mine_game(gameId):
    game_url = "https://www.online-doppelkopf.com/spiele/" + gameId
    print(game_url)
    html_text = requests.get(game_url).text
    soup = BeautifulSoup(html_text, 'html.parser')

    if "Zu viele Anfragen." in soup.text:
        print("Zu viele Anfragen")
        return None

    cards = [[],[],[],[]]
    aufspiel = []
    ansagen = [{},{},{},{}]
    vorbehalt = {}

    # get all rows
    for tr in soup.find_all("tr"):
        if tr.get("bgcolor") == "#f8f8ff":
            for i, vbs in enumerate(tr.find_all("td")):
                if vbs.text is not None:
                    if "Gesund" in vbs.text:
                        vorbehalt[i] = "gesund"
                    elif "Vorbehalt" in vbs.text:
                        vb = list(vbs.parent.next_sibling.find_all("td"))[i].text
                        matches = list(vb_regex.findall(vb))
                        if len(matches) == 0:
                            vorbehalt[i] = "gesund"
                        else:
                            vorbehalt[i] = vorbehalt_map[matches[0]]

        if tr.get("valign") == "bottom":

            # images in row
            for i, image in enumerate(tr.find_all("img")):
                row_num = len(cards[i])

                imageName = image.get("src")
                cardName = imageLinkToCardDict[imageName]
                cards[i].append(cardName)

                # aufspiel
                if image.parent.find("i") is not None:
                    aufspiel.append(i)

                # ansagen
                maybe_ansagen = image.parent.find_all("div")
                for maybe_ansage in maybe_ansagen:
                    if maybe_ansage.find("font") is not None and maybe_ansage.find("font").get("color") == "#b8860b":
                        ansage_str = maybe_ansage.find("font").find("b").string

                        [ansage, id] = list(ansage_regex.match(ansage_str).groups())
                        id = int(id) - 1
                        if row_num in ansagen[i]:
                            ansagen[id][row_num].append(ansagen_map[ansage])
                        else:
                            ansagen[id][row_num] = [ansagen_map[ansage]]


    if len(cards[0]) != 12:
        print("Wrong ruleset")
        return None

    all_b = soup.find_all("b")

    punkte_b = list(filter(lambda f: f.string == "Punkte", all_b))[0]
    tr = punkte_b.parent.parent
    punkte_tds = tr.find_all("td")[1:]
    points = [int(p.string) for p in punkte_tds]
    points = {
        "re": points[0],
        "ko": points[1]
    }

    player_teams = dict((k, v.string[2:-1].lower()) for k, v in enumerate(filter(lambda i: i.string is not None and (" (Ko)" in i.string or " (Re)" in i.string), soup.find_all("i"))))

    return cards, points, ansagen, aufspiel, player_teams, vorbehalt, game_url

def make_test_code(game):
    cardIdsDealt = dict((v, 1) for (k, v) in imageLinkToCardDict.items())

    cardsByPlayer = dict(("player" + str(p + 1), []) for p in range(4))

    cards, points, ansagen, aufspiel, player_teams, vorbehalt, source = game

    code = "Doppelkopf/Doppelkopf.gdl\n\n"
    code += "# " + source + "\n\n"

    # karten geben
    for i, player_cards in enumerate(cards):
        for card in player_cards:
            card_id = cardIdsDealt[card]
            cardIdsDealt[card] += 1

            card_string = card + "_" + str(card_id)
            player_string = "player" + str(i + 1)
            code += "COMMAND\n"
            code += "random (deal " + player_string + " " + card_string + ")\n"
            cardsByPlayer[player_string].append(card_string)

    code += "COMMAND\n"
    code += "random (deal_finish)\n"

    # vorbehalte
    for i in vorbehalt:
        code += "COMMAND\n"
        code += "player" + str(i + 1) + " (vorbehalt " + vorbehalt[i] + ")\n"

    # spielen
    for i in range(len(cards[0])):
        for j in range(4):

            aufspiel_j = (j + aufspiel[i]) % 4
            played_card = cards[aufspiel_j][i]
            player_string = "player" + str(aufspiel_j + 1)

            playerCards = cardsByPlayer[player_string]
            card_string = list(filter(lambda c: c.startswith(played_card), playerCards))[0]
            playerCards.remove(card_string)

            if i in ansagen[aufspiel_j]:
                for ansage in ansagen[aufspiel_j][i]:
                    code += "COMMAND\n"
                    code += player_string + " (" + ansage + ")\n"

            code += "COMMAND\n"
            code += player_string + " (spielen " + card_string + ")\n"

    # auswerten
    code += "COMMAND\n"
    code += "random (auswerten)\n"

    # end state
    code += "PARTIAL_STATE\n"
    code += "(ablauf geben)\n"
    code += "(spiel 2)\n"
    for i in range(4):
        code += "(punkte player" + str(i+1) + " " + str(points[player_teams[i]]) + ")\n"

    return code


def write_test_file(i, test_code):
    f = open("MatchTests/DK_TEST-" + str(i) + ".txt", "w")
    f.write(test_code)
    f.close()


game_ids = mine_all_games()
# game_ids = [ "82.402.772" ]
game_ids += [
    "82.195.829",   # Farbsolo Pik
    "82.397.335",   # Normales Spiel
    "82.187.246",   # Beide verlieren
    "82.378.399",   # Bubensolo
]

for i, game in enumerate(filter(lambda g: g is not None, [mine_game(x) for x in game_ids])):
    test_code = make_test_code(game)
    write_test_file(i, test_code)

