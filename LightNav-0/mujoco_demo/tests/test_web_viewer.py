from html.parser import HTMLParser
from pathlib import Path

WEB_DIR = Path(__file__).parents[1] / "vln_mujoco" / "web"


class ElementIdParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: set[str] = set()

    def handle_starttag(
        self,
        _tag: str,
        attrs: list[tuple[str, str | None]],
    ) -> None:
        for name, value in attrs:
            if name == "id" and value is not None:
                self.ids.add(value)


def test_third_person_picture_in_picture_controls_are_wired() -> None:
    parser = ElementIdParser()
    parser.feed((WEB_DIR / "index.html").read_text(encoding="utf-8"))
    script = (WEB_DIR / "app.js").read_text(encoding="utf-8")
    styles = (WEB_DIR / "styles.css").read_text(encoding="utf-8")

    assert {
        "third-person-overlay",
        "third-person-overlay-image",
        "third-person-overlay-toggle",
    } <= parser.ids
    assert '"/api/third-person.jpg"' in script
    assert "Promise.allSettled(updates)" in script
    assert 'storedBoolean("third-person-overlay-minimized")' in script
    assert ".third-person-overlay.minimized" in styles
    assert "@media (max-width: 560px)" in styles
