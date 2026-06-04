# AI Assistance Disclosure — Frontend

The core of this project was designed and built by the author. This includes
the simulation architecture, the WebGL rendering pipeline, all GLSL shaders
(inline in `renderer.js`), the WebSocket protocol design, the Intelligent
Driver Model (IDM) integration, and the overall frontend and backend structure.

Claude (Anthropic) was used under human supervision to build upon this
foundation — implementing specific features requested by the author (e.g. "add
this panel", "add that view mode") once the core was in place. The author
directed, reviewed, tested, and owns all resulting code.

Each JS file carries a short marker at the top indicating the nature of AI use
for that file.

The HTML files (`app.html`, `index.html`, `loading.html`, `map-selector.html`)
follow the same pattern. The overall page structure, layout, and design language
were established by the author. Claude (Anthropic) was used under human
supervision to implement the HTML and CSS for individual feature sections and
panels as they were added — particularly the larger UI panels inside `app.html`.
The author reviewed and owns all resulting markup.