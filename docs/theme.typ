#set page(
  margin: (x: 1.5cm, y: 1.4cm),
  fill: rgb("#FCFDFE"),
)
#set text(
  font: "Times New Roman",
  size: 10pt,
  fill: rgb("#12263A"),
)
#set par(justify: true, leading: 0.58em)
#set heading(numbering: "1.1")

#let c-ink = rgb("#12263A")
#let c-title = rgb("#0F4C75")
#let c-sub = rgb("#3282B8")
#let c-border = rgb("#B9CCDA")
#let c-soft = rgb("#EEF5FA")
#let c-warn-bg = rgb("#FFF4E8")
#let c-warn-border = rgb("#F0C291")
#let c-ok = rgb("#1B8A5A")
#let c-stop = rgb("#B83737")

#show heading.where(level: 1): it => [
  #v(0.85em)
  #block(
    inset: (x: 10pt, y: 6pt),
    fill: c-soft,
    stroke: (paint: c-border, thickness: 0.9pt),
    radius: 6pt,
  )[
    #text(fill: c-title, size: 15.5pt, weight: "bold")[#it.body]
  ]
]

#show heading.where(level: 2): it => [
  #v(0.4em)
  #text(fill: c-sub, size: 12.2pt, weight: "bold")[#it.body]
]

#let mono(v) = text(font: "Consolas", size: 9.2pt)[`#v`]

#let status-chip(label, tone: "primary") = {
  let fill = if tone == "ok" {
    c-ok
  } else if tone == "warn" {
    rgb("#D97A1E")
  } else if tone == "stop" {
    c-stop
  } else {
    c-title
  }
  box(
    inset: (x: 6pt, y: 2pt),
    fill: fill,
    radius: 999pt,
  )[
    #text(size: 8.5pt, fill: white, weight: "bold")[#label]
  ]
}

#let unit(title, body, tone: "primary") = {
  let fill = if tone == "warn" { c-warn-bg } else { c-soft }
  let stroke = if tone == "warn" { c-warn-border } else { c-border }
  block(
    inset: 8pt,
    radius: 6pt,
    fill: fill,
    stroke: (paint: stroke, thickness: 0.8pt),
  )[
    #text(fill: c-title, weight: "bold")[#title]
    #v(3pt)
    #body
  ]
}

#let concept(label, body) = block(
  inset: (x: 7pt, y: 6pt),
  radius: 5pt,
  fill: white,
  stroke: (paint: c-border, thickness: 0.75pt),
  breakable: false,
)[
  #text(fill: c-sub, weight: "bold", size: 9.5pt)[#label]
  #v(2pt)
  #body
]

#let metric(value, label) = block(
  inset: 7pt,
  radius: 6pt,
  fill: white,
  stroke: (paint: c-border, thickness: 0.75pt),
  breakable: false,
)[
  #text(size: 14pt, weight: "bold", fill: c-title)[#value]
  #linebreak()
  #text(size: 8.8pt, fill: c-sub)[#label]
]
