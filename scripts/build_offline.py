"""Bundle the app into a double-clickable HTML file, without a server."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
html = (root / 'static/index.html').read_text()
css = (root / 'static/style.css').read_text()
css = '\n'.join(line for line in css.splitlines() if not line.startswith('@import'))
html = html.replace('<link rel="stylesheet" href="style.css">', '<style>' + css + '</style>')
for name in ('engine.js', 'app.js'):
    html = html.replace(f'<script src="{name}" defer></script>', '<script>' + (root / ('static/' + name)).read_text().replace('</script', '<\\/script') + '</script>')
# Inline scripts must run after the document has been parsed.
start = html.index('<script>')
end = html.index('</script>', html.index('<script>', start + 8)) + len('</script>')
scripts = html[start:end]
html = html[:start] + html[end:]
html = html.replace('</body>', scripts + '</body>')
html = html.replace('href="index.html"', 'href="#"')
(root / 'EV-Calculus-Lab.html').write_text(html)
(root / 'EV-Calculus-Lab-v3-Learning.html').write_text(html)
print('Built EV-Calculus-Lab.html')
